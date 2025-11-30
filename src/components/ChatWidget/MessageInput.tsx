import React, {useState} from 'react';
import styles from './MessageInput.module.css';

interface MessageInputProps {
  value: string;
  onChange: (value: string) => void;
  onSubmit: (message: string) => void;
  isLoading: boolean;
  error?: string;
  selectedTextMode?: boolean;
  onClearSelection?: () => void;
}

/**
 * Message Input Form
 */
export const MessageInput: React.FC<MessageInputProps> = ({
  value,
  onChange,
  onSubmit,
  isLoading,
  error,
  selectedTextMode = false,
  onClearSelection,
}) => {
  const [localValue, setLocalValue] = useState(value);
  const charCount = localValue.length;
  const maxChars = 2000;
  const isValid = charCount > 0 && charCount <= maxChars;

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (isValid && !isLoading) {
      onSubmit(localValue);
      setLocalValue('');
    }
  };

  const handleChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const newValue = e.target.value;
    if (newValue.length <= maxChars) {
      setLocalValue(newValue);
      onChange(newValue);
    }
  };

  return (
    <form className={styles.messageInput} onSubmit={handleSubmit}>
      <div className={styles.inputWrapper}>
        <textarea
          className={styles.input}
          value={localValue}
          onChange={handleChange}
          onKeyDown={(e) => {
            if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') {
              handleSubmit(e as any);
            }
          }}
          placeholder={selectedTextMode ? "Ask a follow-up question..." : "Ask a question about the book..."}
          disabled={isLoading}
          aria-label="Chat message input"
        />
        <button
          type="submit"
          className={styles.submitButton}
          disabled={!isValid || isLoading}
          aria-label={isLoading ? 'Waiting for response...' : 'Send message'}
        >
          {isLoading ? '⏳' : '➤'}
        </button>
      </div>

      <div
        className={`${styles.charCounter} ${
          charCount >= 1900 ? styles.error : charCount >= 1800 ? styles.warning : ''
        }`}
      >
        {charCount}/{maxChars}
      </div>

      {error && (
        <div className={styles.errorMessage} role="alert">
          {error}
        </div>
      )}
    </form>
  );
};

MessageInput.displayName = 'MessageInput';

MessageInput.displayName = 'MessageInput';
