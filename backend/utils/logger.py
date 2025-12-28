import logging
import sys
from datetime import datetime
from typing import Optional

def setup_logger(name: str = "rag_chatbot", level: int = logging.INFO) -> logging.Logger:
    """
    Creates and configures a structured logger with consistent formatting.

    Args:
        name: Name of the logger
        level: Logging level (default: INFO)

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)

    # Avoid adding multiple handlers if logger already exists
    if logger.handlers:
        return logger

    logger.setLevel(level)

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)

    # Create file handler
    # Using a simple approach - in production, you might want rotation
    file_handler = logging.FileHandler(f"logs/{name}.log", mode='a') if __name__ != "__main__" else logging.StreamHandler(sys.stdout)
    file_handler.setLevel(level)

    # Create formatter with structured format
    formatter = logging.Formatter(
        fmt='%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

    console_handler.setFormatter(formatter)
    file_handler.setFormatter(formatter)

    # Add handlers to logger
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)

    # Prevent propagation to root logger to avoid duplicate logs
    logger.propagate = False

    return logger

# Create a default logger instance
logger = setup_logger()

def log_info(message: str, extra: Optional[dict] = None):
    """Log an info message with optional extra context."""
    if extra:
        logger.info(message, extra=extra)
    else:
        logger.info(message)

def log_error(message: str, extra: Optional[dict] = None):
    """Log an error message with optional extra context."""
    if extra:
        logger.error(message, extra=extra)
    else:
        logger.error(message)

def log_warning(message: str, extra: Optional[dict] = None):
    """Log a warning message with optional extra context."""
    if extra:
        logger.warning(message, extra=extra)
    else:
        logger.warning(message)

def log_debug(message: str, extra: Optional[dict] = None):
    """Log a debug message with optional extra context."""
    if extra:
        logger.debug(message, extra=extra)
    else:
        logger.debug(message)

# Ensure logs directory exists
import os
if not os.path.exists('logs'):
    os.makedirs('logs')