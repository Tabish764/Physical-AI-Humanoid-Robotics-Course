import React, {useState} from 'react';
import {useChat} from '../../hooks/useChat';
import {useSelectedText} from '../../hooks/useSelectedText';
import {ChatIcon} from './ChatIcon';
import {ChatModal} from './ChatModal';
import {SelectionButton} from './SelectionButton';
import styles from './ChatWidget.module.css';

/**
 * ChatWidget component
 * 
 * Main container component for the RAG chat interface.
 */
export const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [inputValue, setInputValue] = useState('');
  const [selectedTextMode, setSelectedTextMode] = useState(false);
  const [preservedSelectedText, setPreservedSelectedText] = useState<string | null>(null);
  const chat = useChat();
  const selectedText = useSelectedText();

  const handleIconClick = () => {
    setIsOpen(!isOpen);
    setSelectedTextMode(false);
    setPreservedSelectedText(null);
  };

  const handleClose = () => {
    setIsOpen(false);
    setSelectedTextMode(false);
    setPreservedSelectedText(null);
  };

  const handleSelectionButtonClick = () => {
    // Preserve the selected text before opening modal
    // This ensures it persists even if browser selection changes
    if (selectedText.selectedText) {
      setPreservedSelectedText(selectedText.selectedText);
      setSelectedTextMode(true);
      setIsOpen(true);
    }
  };

  const handleClearSelection = () => {
    selectedText.clearSelection();
    setSelectedTextMode(false);
    setPreservedSelectedText(null);
  };

  const handleSubmit = (question: string) => {
    // Use preserved text if in selected text mode, otherwise use current selection
    const textToUse = selectedTextMode && preservedSelectedText 
      ? preservedSelectedText 
      : selectedText.selectedText;
    
    if (selectedTextMode && textToUse) {
      chat.sendMessageWithContext(question, textToUse);
    } else {
      chat.sendMessage(question);
    }
    setInputValue('');
  };

  return (
    <div className={styles.chatWidget}>
      <ChatIcon onClick={handleIconClick} isOpen={isOpen} />
      {selectedText.isSelectionActive && 
       selectedText.position && 
       !isOpen && (
        <SelectionButton
          position={selectedText.position}
          onClick={handleSelectionButtonClick}
        />
      )}
      {isOpen && (
        <ChatModal
          isOpen={isOpen}
          onClose={handleClose}
          messages={chat.messages}
          isLoading={chat.isLoading}
          error={chat.error}
          inputValue={inputValue}
          onInputChange={setInputValue}
          onSubmit={handleSubmit}
          selectedText={selectedTextMode ? preservedSelectedText : null}
          onClearSelection={selectedTextMode ? handleClearSelection : undefined}
        />
      )}
    </div>
  );
};

ChatWidget.displayName = 'ChatWidget';
