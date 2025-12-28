# Quickstart: Structured Logging System for RAG Chatbot Backend

## Overview
This guide explains how to set up and use the structured logging system for the RAG Chatbot backend.

## Setup
The logging system is implemented in `utils/logger.py` and can be used throughout the application.

## Basic Usage

### Import and Initialize
```python
from utils.logger import setup_logger, logger

# Initialize with default settings
logger = setup_logger()
```

### Using Helper Functions
```python
from utils.logger import log_info, log_error, log_warning, log_debug

log_info("Application started successfully")
log_warning("This is a warning message")
log_error("An error occurred")
log_debug("Debug information")
```

### Custom Logger Instance
```python
from utils.logger import setup_logger

# Create a custom logger with specific name and level
custom_logger = setup_logger(name="my_module", level=logging.DEBUG)
```

## Configuration
- Log format: `%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s`
- Date format: `YYYY-MM-DD HH:MM:SS`
- File output: `logs/{name}.log` (if logs directory exists)
- Console output: Always enabled

## Directory Safety
The system checks for the existence of the `logs/` directory before attempting to create file handlers. If the directory doesn't exist, only console logging will be enabled to prevent crashes in production environments.

## Best Practices
1. Use appropriate log levels (INFO for general information, ERROR for errors, etc.)
2. Include relevant context in log messages
3. Avoid logging sensitive information
4. Use the helper functions for consistency