import React from 'react';
import styles from './ChatIcon.module.css';

interface ChatIconProps {
  onClick: () => void;
  isOpen: boolean;
}

/**
 * Floating Chat Icon Button
 */
export const ChatIcon: React.FC<ChatIconProps> = ({onClick, isOpen}) => {
  return (
    <button
      className={styles.chatIcon}
      onClick={onClick}
      aria-label={isOpen ? 'Close chat' : 'Open chat'}
      aria-pressed={isOpen}
    >
      <span className={styles.icon}>{isOpen ? '✕' : '💬'}</span>
    </button>
  );
};

ChatIcon.displayName = 'ChatIcon';
