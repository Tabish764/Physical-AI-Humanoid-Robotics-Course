import logging
import sys
import os
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

    # Create console handler (always active)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)

    # Create formatter with structured format
    formatter = logging.Formatter(
        fmt='%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    console_handler.setFormatter(formatter)

    # Check if logs directory exists, create it if possible
    logs_dir_exists = os.path.exists('logs')
    if not logs_dir_exists:
        try:
            os.makedirs('logs', exist_ok=True)
            logs_dir_exists = True
        except (PermissionError, OSError):
            # If we can't create the logs directory, continue with console only
            logs_dir_exists = False

    # Create file handler only if logs directory exists and is accessible
    if logs_dir_exists:
        try:
            file_handler = logging.FileHandler(f"logs/{name}.log", mode='a')
            file_handler.setLevel(level)
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)
        except (PermissionError, OSError):
            # If we can't create the file handler, continue with console only
            pass

    # Add console handler to logger
    logger.addHandler(console_handler)

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