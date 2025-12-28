# Data Model: Structured Logging System for RAG Chatbot Backend

## Log Message Entity

**Description**: Represents a single log entry with structured information

**Fields**:
- timestamp: String (formatted as YYYY-MM-DD HH:MM:SS)
- logger_name: String (name of the logger instance)
- level: String (log level - INFO, ERROR, WARNING, DEBUG)
- filename: String (source file where the log was generated)
- line_number: Integer (line number in the source file)
- message: String (the actual log message content)

**Validation Rules**:
- timestamp must follow YYYY-MM-DD HH:MM:SS format
- level must be one of: INFO, ERROR, WARNING, DEBUG
- message must not be empty
- filename must be a valid file path string

## Logger Configuration Entity

**Description**: Represents the configuration for a logger instance

**Fields**:
- name: String (name of the logger, default "rag_chatbot")
- level: Integer (logging level constant, default logging.INFO)
- has_file_handler: Boolean (indicates if file handler is active)
- has_console_handler: Boolean (indicates if console handler is active)
- log_file_path: String (path to log file if file handler is active, optional)

**Validation Rules**:
- name must be a valid string identifier
- level must be a valid logging level constant
- log_file_path must be a valid file path if provided

## Log Level Entity

**Description**: Represents the different logging levels supported

**Values**:
- INFO: Informational messages
- ERROR: Error messages
- WARNING: Warning messages
- DEBUG: Debug messages

**Constraints**:
- Each level has a specific priority order (DEBUG < INFO < WARNING < ERROR)
- Messages below the configured level threshold will not be logged