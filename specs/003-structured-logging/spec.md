# Feature Specification: Structured Logging System for RAG Chatbot Backend

**Feature Branch**: `003-structured-logging`
**Created**: 2025-12-29
**Status**: Draft
**Input**: User description: "1. Objective

To implement a robust, structured logging system for the RAG Chatbot backend that:

Logs to both console and file (locally)

Works safely in production (Render)

Avoids errors if directories or files don't exist

Supports multiple log levels (INFO, ERROR, WARNING, DEBUG)

2. Non-Goals

Logging rotation (optional for now)

External logging services (e.g., Sentry)

JSON structured logs (plain text is enough for now)

3. Requirements
3.1 Functional Requirements

Logger Creation

Function: setup_logger(name: str = \"rag_chatbot\", level: int = logging.INFO) -> logging.Logger

Creates a logger with console handler and file handler.

File handler logs to logs/{name}.log if the folder exists.

Handlers

Console handler: always logs to stdout.

File handler: logs to file only if logs/ exists. If not, skip file logging.

Formatter

Format: %(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s

Date format: YYYY-MM-DD HH:MM:SS

Log Levels

Support INFO, ERROR, WARNING, DEBUG.

Default Logger Instance

A default logger instance logger should be created automatically.

Helper Functions

log_info(message: str, extra: Optional[dict] = None)

log_error(message: str, extra: Optional[dict] = None)

log_warning(message: str, extra: Optional[dict] = None)

log_debug(message: str, extra: Optional[dict] = None)

Directory Safety

Ensure logs/ folder exists before creating a FileHandler.

If folder does not exist, console logging still works.

3.2 Non-Functional Requirements

Must not crash the application if the log folder is missing.

Must avoid duplicate handlers when the logger is imported multiple times.

Must work on local development and Render deployment.

Must be compatible with Python 3.11.

4. Success Criteria

On local machine:

logs/rag_chatbot.log is created.

Logs appear both in console and file.

On Render:

No FileNotFoundError.

Logs appear in Render's stdout logs.

Test all levels:

log_info(\"Info message\")
log_warning(\"Warning message\")
log_error(\"Error message\")
log_debug(\"Debug message\")


Running multiple times does not create duplicate log entries.

5. Non-Goals

Logging rotation

Remote logging

JSON formatting

Sensitive information redaction

6. Tasks

Create utils/logger.py.

Ensure logs/ folder creation is safe.

Implement setup_logger function.

Add helper functions: log_info, log_error, log_warning, log_debug.

Test locally and on Render.

Document usage in README."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Backend Developer accesses structured logs (Priority: P1)

As a backend developer, I need to access structured logs from the RAG Chatbot application so that I can troubleshoot issues, monitor application behavior, and maintain system reliability. The logging system must work both in local development and in production environments without causing crashes.

**Why this priority**: This is the core functionality that enables all debugging and monitoring capabilities. Without proper logging, it's impossible to diagnose issues in the application.

**Independent Test**: The developer can run the application locally and see logs appearing both in the console and in a log file (if the logs directory exists). In production, the application runs without errors even if the logs directory doesn't exist.

**Acceptance Scenarios**:

1. **Given** the application is running locally, **When** an INFO level message is logged, **Then** the message appears in both console and the log file with proper formatting
2. **Given** the application is running in production (Render), **When** the logs directory doesn't exist, **Then** the application continues to run without crashing and logs to console only

---

### User Story 2 - Operations team monitors application health (Priority: P2)

As an operations team member, I need to monitor application health through different log levels so that I can quickly identify warnings, errors, and debug information when needed for system maintenance.

**Why this priority**: Different log levels allow for effective filtering of information based on urgency and importance, enabling efficient system monitoring.

**Independent Test**: The system can log messages at INFO, ERROR, WARNING, and DEBUG levels with appropriate severity indicators.

**Acceptance Scenarios**:

1. **Given** the logging system is configured, **When** different log level messages are generated, **Then** each message is properly tagged with its severity level
2. **Given** a log file exists, **When** multiple log levels are written, **Then** the log file contains all messages with clear level indicators

---

### User Story 3 - System maintains stability during logging operations (Priority: P3)

As a system administrator, I need the application to remain stable even when logging encounters issues (missing directories, permission errors) so that logging problems don't affect core application functionality.

**Why this priority**: Logging should be a non-critical service that doesn't impact the primary functions of the application when it encounters issues.

**Independent Test**: The application continues to function normally even when log directories don't exist or can't be written to.

**Acceptance Scenarios**:

1. **Given** the logs directory doesn't exist, **When** the application starts, **Then** it runs without errors and logs to console only
2. **Given** the application is running, **When** the logs directory is removed during operation, **Then** the application continues to run and only logs to console

---

### Edge Cases

- What happens when the logs directory is created after the application starts but doesn't exist initially? The system should begin file logging once the directory exists.
- How does the system handle multiple imports of the same logger module? It should avoid duplicate log entries.
- What happens when the disk is full and log files can't be written? The system should continue operating and only log to console.
- How does the system behave when file permissions prevent writing to log files? The system should continue operating and only log to console.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a setup_logger function that accepts a name and level parameter and returns a configured logger instance
- **FR-002**: System MUST create both console and file handlers for the logger when the logs directory exists
- **FR-003**: System MUST create only console handler when the logs directory doesn't exist, without crashing
- **FR-004**: System MUST format log messages with timestamp, logger name, level, filename, line number, and message
- **FR-005**: System MUST support INFO, ERROR, WARNING, and DEBUG log levels
- **FR-006**: System MUST provide helper functions: log_info, log_error, log_warning, log_debug for easy logging
- **FR-007**: System MUST check for the existence of logs/ directory before attempting to create file handlers
- **FR-008**: System MUST prevent duplicate handlers when the logger is imported multiple times
- **FR-009**: System MUST use the format: %(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s
- **FR-010**: System MUST use date format: YYYY-MM-DD HH:MM:SS

### Key Entities

- **Logger Instance**: Represents a configured logging object that can write to both console and file
- **Log Message**: Contains timestamp, level, source location, and message content
- **Log File**: File in the logs/ directory containing formatted log messages
- **Log Handler**: Component responsible for directing log messages to output destinations (console or file)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: On local machine, the logs/rag_chatbot.log file is created successfully when the logs directory exists
- **SC-002**: On production (Render), the application runs without FileNotFoundError when logs directory doesn't exist
- **SC-003**: Log messages appear in both console and file with proper formatting including timestamp, name, level, and source location
- **SC-004**: All log levels (INFO, ERROR, WARNING, DEBUG) can be successfully logged and distinguished
- **SC-005**: Running the application multiple times does not create duplicate log entries
- **SC-006**: The application continues to function normally even when logging to file fails
- **SC-007**: The logging system is compatible with Python 3.11 and works in both local and production environments