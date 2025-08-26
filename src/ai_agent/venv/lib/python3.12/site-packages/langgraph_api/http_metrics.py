from collections import defaultdict
from typing import Any

FILTERED_ROUTES = {"/ok", "/info", "/metrics", "/docs", "/openapi.json"}

MAX_REQUEST_COUNT_ENTRIES = 5000
MAX_HISTOGRAM_ENTRIES = 1000


def get_route(route: Any) -> str | None:
    try:
        # default lg api routes use the custom APIRoute where scope["route"] is set to a string
        if isinstance(route, str):
            return route
        else:
            # custom FastAPI routes provided by user_router attach an object to scope["route"]
            route_path = getattr(route, "path", None)
            return route_path
    except Exception:
        return None


def should_filter_route(route_path: str) -> bool:
    # use endswith to honor MOUNT_PREFIX
    return any(route_path.endswith(suffix) for suffix in FILTERED_ROUTES)


class HTTPMetricsCollector:
    def __init__(self):
        # Counter: Key: (method, route, status), Value: count
        self._request_counts: dict[tuple[str, str, int], int] = defaultdict(int)

        self._histogram_buckets = [
            0.01,
            0.1,
            0.5,
            1,
            5,
            15,
            30,
            60,
            120,
            300,
            600,
            1800,
            3600,
            float("inf"),
        ]
        self._histogram_bucket_labels = [
            "+Inf" if value == float("inf") else str(value)
            for value in self._histogram_buckets
        ]

        self._histogram_data: dict[tuple[str, str], dict] = defaultdict(
            lambda: {
                "bucket_counts": [0] * len(self._histogram_buckets),
                "sum": 0.0,
                "count": 0,
            }
        )

    def record_request(
        self, method: str, route: Any, status: int, latency_ms: float
    ) -> None:
        route_path = get_route(route)
        if route_path is None:
            return

        if should_filter_route(route_path):
            return

        request_count_key = (method, route_path, status)
        histogram_key = (method, route_path)

        if (
            request_count_key not in self._request_counts
            and len(self._request_counts) >= MAX_REQUEST_COUNT_ENTRIES
        ):
            return

        if (
            histogram_key not in self._histogram_data
            and len(self._histogram_data) >= MAX_HISTOGRAM_ENTRIES
        ):
            return

        self._request_counts[request_count_key] += 1

        latency_seconds = latency_ms / 1000.0
        hist_data = self._histogram_data[histogram_key]

        for i, bucket_value in enumerate(self._histogram_buckets):
            if latency_seconds <= bucket_value:
                hist_data["bucket_counts"][i] += 1
                break

        hist_data["sum"] += latency_seconds
        hist_data["count"] += 1

    def get_metrics(
        self, project_id: str, revision_id: str, format: str = "prometheus"
    ) -> dict | list[str]:
        if format == "json":
            return {
                "api": {
                    "http_requests_total": [
                        {
                            "method": method,
                            "path": path,
                            "status": status,
                            "count": count,
                        }
                        for (
                            method,
                            path,
                            status,
                        ), count in self._request_counts.items()
                    ]
                }
            }

        metrics = []

        # Counter metrics
        if self._request_counts:
            metrics.extend(
                [
                    "# HELP lg_api_http_requests_total Total number of HTTP requests.",
                    "# TYPE lg_api_http_requests_total counter",
                ]
            )

            for (method, path, status), count in self._request_counts.items():
                metrics.append(
                    f'lg_api_http_requests_total{{project_id="{project_id}", revision_id="{revision_id}", method="{method}", path="{path}", status="{status}"}} {count}'
                )

        # Histogram metrics
        if self._histogram_data:
            metrics.extend(
                [
                    "# HELP lg_api_http_requests_latency_seconds HTTP request latency in seconds.",
                    "# TYPE lg_api_http_requests_latency_seconds histogram",
                ]
            )

            for (method, path), hist_data in self._histogram_data.items():
                acc = 0
                for i, bucket_count in enumerate(hist_data["bucket_counts"]):
                    acc += bucket_count
                    bucket_label = self._histogram_bucket_labels[i]
                    metrics.append(
                        f'lg_api_http_requests_latency_seconds_bucket{{project_id="{project_id}", revision_id="{revision_id}", method="{method}", path="{path}", le="{bucket_label}"}} {acc}'
                    )

                metrics.extend(
                    [
                        f'lg_api_http_requests_latency_seconds_sum{{project_id="{project_id}", revision_id="{revision_id}", method="{method}", path="{path}"}} {hist_data["sum"]:.6f}',
                        f'lg_api_http_requests_latency_seconds_count{{project_id="{project_id}", revision_id="{revision_id}", method="{method}", path="{path}"}} {hist_data["count"]}',
                    ]
                )

        return metrics


HTTP_METRICS_COLLECTOR = HTTPMetricsCollector()
