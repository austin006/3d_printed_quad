# Makefile for Drone Control Agent

.PHONY: help install check run test docker-build docker-run clean benchmark

# Default target
help:
	@echo "Drone Control Agent - Available commands:"
	@echo "  make install     - Install Python dependencies"
	@echo "  make check       - Check system compatibility"
	@echo "  make run         - Run the agent (requires Ollama running)"
	@echo "  make test        - Run automated test sequence"
	@echo "  make benchmark   - Run performance benchmark"
	@echo "  make docker-build - Build Docker image"
	@echo "  make docker-run  - Run with Docker Compose"
	@echo "  make clean       - Clean temporary files"

# Install dependencies
install:
	pip3 install -r requirements.txt
	@echo "✅ Dependencies installed"

# Check compatibility
check:
	@echo "🔍 Checking system compatibility..."
	python3 check_compatibility.py

# Run the agent
run: check
	@echo "🚁 Starting Drone Control Agent..."
	@bash launch_agent.sh

# Run tests
test:
	@echo "🧪 Running test sequence..."
	python3 test_agent.py

# Performance benchmark
benchmark:
	@echo "⚡ Running performance benchmark..."
	@echo "Command processing times:"
	@time -p python3 -c "import test_agent; test_agent.main()" 2>&1 | grep real

# Docker operations
docker-build:
	docker build -t drone-agent .
	@echo "✅ Docker image built"

docker-run:
	docker-compose up -d
	@echo "✅ Services started. Run 'docker attach drone-control-agent' to interact"

docker-stop:
	docker-compose down
	@echo "✅ Services stopped"

# Clean temporary files
clean:
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete
	@echo "✅ Cleaned temporary files"

# Development helpers
format:
	black agent.py drone_tools.py test_agent.py
	@echo "✅ Code formatted"

lint:
	pylint agent.py drone_tools.py
	@echo "✅ Linting complete"