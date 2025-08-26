import langgraph.version
from starlette.responses import JSONResponse, PlainTextResponse

from langgraph_api import __version__, config, metadata
from langgraph_api.http_metrics import HTTP_METRICS_COLLECTOR
from langgraph_api.route import ApiRequest
from langgraph_license.validation import plus_features_enabled
from langgraph_runtime.database import connect, pool_stats
from langgraph_runtime.metrics import get_metrics
from langgraph_runtime.ops import Runs

METRICS_FORMATS = {"prometheus", "json"}


async def meta_info(request: ApiRequest):
    plus = plus_features_enabled()
    return JSONResponse(
        {
            "version": __version__,
            "langgraph_py_version": langgraph.version.__version__,
            "flags": {
                "assistants": True,
                "crons": plus and config.FF_CRONS_ENABLED,
                "langsmith": bool(config.LANGSMITH_API_KEY) and bool(config.TRACING),
                "langsmith_tracing_replicas": True,
            },
            "host": {
                "kind": metadata.HOST,
                "project_id": metadata.PROJECT_ID,
                "host_revision_id": metadata.HOST_REVISION_ID,
                "revision_id": metadata.REVISION,
                "tenant_id": metadata.TENANT_ID,
            },
        }
    )


async def meta_metrics(request: ApiRequest):
    # determine output format
    metrics_format = request.query_params.get("format", "prometheus")
    if metrics_format not in METRICS_FORMATS:
        metrics_format = "prometheus"

    # collect stats
    metrics = get_metrics()
    worker_metrics = metrics["workers"]
    workers_max = worker_metrics["max"]
    workers_active = worker_metrics["active"]
    workers_available = worker_metrics["available"]

    http_metrics = HTTP_METRICS_COLLECTOR.get_metrics(
        metadata.PROJECT_ID, metadata.HOST_REVISION_ID, metrics_format
    )

    if metrics_format == "json":
        async with connect() as conn:
            resp = {
                **pool_stats(),
                "queue": await Runs.stats(conn),
                **http_metrics,
            }
            if config.N_JOBS_PER_WORKER > 0:
                resp["workers"] = worker_metrics
            return JSONResponse(resp)
    elif metrics_format == "prometheus":
        async with connect() as conn:
            queue_stats = await Runs.stats(conn)

            metrics = [
                "# HELP lg_api_num_pending_runs The number of runs currently pending.",
                "# TYPE lg_api_num_pending_runs gauge",
                f'lg_api_num_pending_runs{{project_id="{metadata.PROJECT_ID}", revision_id="{metadata.HOST_REVISION_ID}"}} {queue_stats["n_pending"]}',
                "# HELP lg_api_num_running_runs The number of runs currently running.",
                "# TYPE lg_api_num_running_runs gauge",
                f'lg_api_num_running_runs{{project_id="{metadata.PROJECT_ID}", revision_id="{metadata.HOST_REVISION_ID}"}} {queue_stats["n_running"]}',
            ]

            if config.N_JOBS_PER_WORKER > 0:
                metrics.extend(
                    [
                        "# HELP lg_api_workers_max The maximum number of workers available.",
                        "# TYPE lg_api_workers_max gauge",
                        f'lg_api_workers_max{{project_id="{metadata.PROJECT_ID}", revision_id="{metadata.HOST_REVISION_ID}"}} {workers_max}',
                        "# HELP lg_api_workers_active The number of currently active workers.",
                        "# TYPE lg_api_workers_active gauge",
                        f'lg_api_workers_active{{project_id="{metadata.PROJECT_ID}", revision_id="{metadata.HOST_REVISION_ID}"}} {workers_active}',
                        "# HELP lg_api_workers_available The number of available (idle) workers.",
                        "# TYPE lg_api_workers_available gauge",
                        f'lg_api_workers_available{{project_id="{metadata.PROJECT_ID}", revision_id="{metadata.HOST_REVISION_ID}"}} {workers_available}',
                    ]
                )

            metrics.extend(http_metrics)

        metrics_response = "\n".join(metrics)
        return PlainTextResponse(metrics_response)
