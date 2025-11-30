import React from 'react';
import styles from './Message.module.css';
import type {ChatMessage} from '../../types/chat';

interface MessageProps {
  message: ChatMessage;
}

/**
 * Message Component
 */
export const Message: React.FC<MessageProps> = ({message}) => {
  const isUser = message.role === 'user';
  const isError = message.error;

  return (
    <article
      className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage} ${
        isError ? styles.errorMessage : ''
      }`}
      role={isError ? 'alert' : 'article'}
    >
      <div className={styles.content}>{message.content}</div>

      {message.timestamp && (
        <div className={styles.timestamp}>
          {new Date(message.timestamp).toLocaleTimeString()}
        </div>
      )}

      {message.sources && message.sources.length > 0 && (
        <div className={styles.sources}>
          <strong>Sources:</strong>
          <ul>
            {message.sources.map((source, idx) => (
              <li key={`${source}-${idx}`}>
                <a
                  href={source}
                  target="_blank"
                  rel="noopener noreferrer"
                  title={source}
                >
                  Source {idx + 1}
                </a>
              </li>
            ))}
          </ul>
        </div>
      )}
    </article>
  );
};

Message.displayName = 'Message';

Message.displayName = 'Message';
