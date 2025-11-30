/**
 * useSelectedText - Custom React Hook for Text Selection Detection
 * Detects selected text on the page and provides position information
 * @module hooks/useSelectedText
 */

import { useState, useEffect, useCallback } from 'react';

/**
 * Return type for useSelectedText hook
 * @interface UseSelectedTextReturn
 */
export interface UseSelectedTextReturn {
  selectedText: string | null;
  isSelectionActive: boolean;
  position: { x: number; y: number } | null;
  clearSelection: () => void;
}

/**
 * Custom hook for detecting selected text on the page
 * Listens for selection changes and provides the selected text
 * and position information
 *
 * @returns {UseSelectedTextReturn} Selected text and position
 */
export function useSelectedText(): UseSelectedTextReturn {
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [position, setPosition] = useState<{ x: number; y: number } | null>(null);

  /**
   * Handles text selection and extracts selection info
   */
  const handleSelection = useCallback(() => {
    try {
      const selection = window.getSelection();

      // Ignore selections inside the chat modal or input fields
      const activeElement = document.activeElement;
      if (activeElement) {
        const isInput = activeElement.tagName === 'INPUT' || 
                       activeElement.tagName === 'TEXTAREA' ||
                       activeElement.closest('[role="dialog"]') !== null ||
                       activeElement.closest('.chatModal') !== null;
        if (isInput) {
          // Don't clear existing selection when clicking inside modal
          return;
        }
      }

      if (!selection || selection.toString().length === 0) {
        // Only clear if selection is truly empty and not inside modal
        const isInsideModal = document.activeElement?.closest('[role="dialog"]') !== null ||
                             document.activeElement?.closest('.chatModal') !== null;
        if (!isInsideModal) {
          setSelectedText(null);
          setPosition(null);
        }
        return;
      }

      const text = selection.toString().trim();

      // Ignore selections that are too short (single words or punctuation)
      if (text.length < 2) {
        setSelectedText(null);
        setPosition(null);
        return;
      }

      // Get selection range for positioning
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();

      // Calculate button position (above the selection, centered)
      const buttonWidth = 100; // Approximate button width
      const buttonX = rect.left + rect.width / 2; // Center horizontally
      let buttonY = rect.top - 10; // 10px above selection

      // Handle edge case: selection near top of viewport
      if (buttonY < 60) {
        buttonY = rect.bottom + 10; // Show below selection instead
      }

      // Handle edge case: selection near right edge
      if (buttonX + buttonWidth / 2 > window.innerWidth - 20) {
        // Adjust to fit within viewport
        const adjustedX = window.innerWidth - buttonWidth / 2 - 20;
        setSelectedText(text);
        setPosition({ x: adjustedX, y: buttonY });
        return;
      }

      // Handle edge case: selection near left edge
      if (buttonX - buttonWidth / 2 < 20) {
        const adjustedX = buttonWidth / 2 + 20;
        setSelectedText(text);
        setPosition({ x: adjustedX, y: buttonY });
        return;
      }

      setSelectedText(text);
      setPosition({ x: buttonX, y: buttonY });
    } catch (error) {
      // Silently fail on cross-element or complex selections
      console.debug('Selection detection error:', error);
    }
  }, []);

  /**
   * Clears the current selection
   */
  const clearSelection = useCallback(() => {
    setSelectedText(null);
    setPosition(null);
    if (window.getSelection()) {
      window.getSelection()?.removeAllRanges();
    }
  }, []);

  /**
   * Set up event listeners for selection changes
   */
  useEffect(() => {
    const handleMouseUp = () => {
      // Defer to allow text selection to complete
      setTimeout(() => {
        handleSelection();
      }, 50);
    };

    const handleTouchEnd = () => {
      // Defer to allow text selection to complete on mobile
      setTimeout(() => {
        handleSelection();
      }, 100);
    };

    const handleSelectionChange = () => {
      handleSelection();
    };

    // Listen for selection events
    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('touchend', handleTouchEnd);
    document.addEventListener('selectionchange', handleSelectionChange);

    // Cleanup
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('touchend', handleTouchEnd);
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, [handleSelection]);

  return {
    selectedText,
    isSelectionActive: selectedText !== null,
    position,
    clearSelection,
  };
}
