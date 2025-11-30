import React from 'react';
import styles from './SelectionButton.module.css';

interface SelectionButtonProps {
  position: { x: number; y: number };
  onClick: () => void;
}

/**
 * SelectionButton component
 * 
 * Small floating button that appears near selected text to allow
 * users to ask AI about the selection.
 */
export const SelectionButton: React.FC<SelectionButtonProps> = ({
  position,
  onClick,
}) => {
  return (
    <button
      className={styles.selectionButton}
      onClick={onClick}
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
      }}
      aria-label="Ask AI about selected text"
      type="button"
    >
      <span className={styles.icon}>✨</span>
      <span className={styles.text}>Ask AI</span>
    </button>
  );
};

SelectionButton.displayName = 'SelectionButton';

