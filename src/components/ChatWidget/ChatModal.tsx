import React, {useEffect} from 'react';
import {MessageList} from './MessageList';
import {MessageInput} from './MessageInput';
import styles from './ChatModal.module.css';
import type {ChatMessage} from '../../types/chat';

interface ChatModalProps {
  isOpen: boolean;
  onClose: () => void;
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;
  inputValue: string;
  onInputChange: (value: string) => void;
  onSubmit: (question: string) => void;
  selectedText?: string | null;
  onClearSelection?: () => void;
}

/**
 * Chat Modal Container
 */
export const ChatModal: React.FC<ChatModalProps> = ({
  isOpen,
  onClose,
  messages,
  isLoading,
  error,
  inputValue,
  onInputChange,
  onSubmit,
  selectedText,
  onClearSelection,
}) => {
  useEffect(() => {
    if (!isOpen) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, onClose]);

  if (!isOpen) return null;

  return (
    <>
      <div className={styles.backdrop} onClick={onClose} aria-hidden="true" />
      <div
        className={styles.chatModal}
        role="dialog"
        aria-modal="true"
        aria-label="Chat with AI about the book"
      >
        <div className={styles.header}>
          <h2 className={styles.title}>Chat with AI</h2>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close chat"
          >
            ✕
          </button>
        </div>

        {selectedText && (
          <div className={styles.selectedTextContext}>
            <div className={styles.selectedTextHeader}>
              <span className={styles.selectedTextLabel}>Selected text:</span>
              {onClearSelection && (
                <button
                  className={styles.clearSelectionButton}
                  onClick={onClearSelection}
                  aria-label="Clear selection"
                  type="button"
                >
                  Clear
                </button>
              )}
            </div>
            <div className={styles.selectedTextContent}>
              {selectedText}
            </div>
          </div>
        )}

        <MessageList
          messages={messages}
          isLoading={isLoading}
        />

        {error && (
          <div className={styles.errorMessage} role="alert">
            {error}
          </div>
        )}

        <div className={styles.footer}>
          <MessageInput
            value={inputValue}
            onChange={onInputChange}
            onSubmit={onSubmit}
            isLoading={isLoading}
            error={error || undefined}
            selectedTextMode={!!selectedText}
            onClearSelection={onClearSelection}
          />
        </div>
      </div>
    </>
  );
};

ChatModal.displayName = 'ChatModal';
