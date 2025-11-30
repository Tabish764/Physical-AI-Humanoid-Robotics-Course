import React, {useEffect, useRef} from 'react';
import {Message} from './Message';
import styles from './MessageList.module.css';
import type {ChatMessage} from '../../types/chat';

interface MessageListProps {
  messages: ChatMessage[];
  isLoading: boolean;
}

/**
 * Message List Container
 */
export const MessageList: React.FC<MessageListProps> = ({messages, isLoading}) => {
  const endRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    endRef.current?.scrollIntoView({behavior: 'smooth'});
  }, [messages, isLoading]);

  return (
    <div
      className={styles.messageList}
      role="log"
      aria-live="polite"
      aria-label="Chat message history"
    >
      {messages.length === 0 && !isLoading && (
        <div className={styles.emptyState}>
          Start a conversation by asking a question about the book!
        </div>
      )}

      {messages.map((message) => (
        <Message key={message.id} message={message} />
      ))}

      {isLoading && (
        <div className={styles.loadingIndicator} aria-label="AI is thinking...">
          <div className={styles.spinner} />
        </div>
      )}

      <div ref={endRef} />
    </div>
  );
};

MessageList.displayName = 'MessageList';

MessageList.displayName = 'MessageList';
