"""
Example usage of the structured logging system.
"""

from utils.logger import setup_logger, logger, log_info, log_error, log_warning, log_debug


def example_basic_usage():
    """Example of basic logger usage."""
    print("=== Basic Logger Usage ===")

    # Use the default logger
    log_info("Application starting up")
    log_warning("This is a warning message")
    log_error("This is an error message")
    log_debug("This is a debug message")

    print("Basic usage completed - check console and log file")


def example_custom_logger():
    """Example of creating and using a custom logger."""
    print("\n=== Custom Logger Usage ===")

    # Create a custom logger for a specific module
    my_logger = setup_logger("my_module", level=10)  # 10 = DEBUG level

    my_logger.info("Info message from custom logger")
    my_logger.warning("Warning from custom logger")
    my_logger.error("Error from custom logger")
    my_logger.debug("Debug from custom logger")

    print("Custom logger usage completed")


def example_with_extra_context():
    """Example of logging with extra context."""
    print("\n=== Logger with Extra Context ===")

    # Log with extra context
    log_info("User login attempt", extra={"user_id": 123, "ip_address": "192.168.1.1"})
    log_error("Database connection failed", extra={"host": "localhost", "port": 5432})
    log_warning("High memory usage", extra={"memory_percent": 85.5, "process_id": 1234})

    print("Extra context logging completed")


def example_different_levels():
    """Example of different logging levels."""
    print("\n=== Different Log Levels ===")

    # Create a logger with WARNING level
    warning_logger = setup_logger("warning_test", level=30)  # 30 = WARNING level

    # These will be logged (WARNING and above)
    warning_logger.warning("This warning will be logged")
    warning_logger.error("This error will be logged")

    # This will NOT be logged (below WARNING level)
    warning_logger.info("This info will NOT appear (below WARNING level)")
    warning_logger.debug("This debug will NOT appear (below WARNING level)")

    print("Log level filtering example completed")


if __name__ == "__main__":
    print("Structured Logging System - Example Usage")
    print("=" * 50)

    example_basic_usage()
    example_custom_logger()
    example_with_extra_context()
    example_different_levels()

    print("\n" + "=" * 50)
    print("All examples completed!")
    print("Check console output and logs/test.log for details.")