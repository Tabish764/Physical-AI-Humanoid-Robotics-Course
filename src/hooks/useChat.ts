/**
 * useChat - Custom React Hook for Chat Widget State Management
 * Manages conversation state, API calls, and message history
 * @module hooks/useChat
 */

import { useState, useCallback, useEffect } from 'react';
import { ChatMessage, Conversation, ChatWidgetState } from '../types/chat';
import { postChat, postChatSelected, APIError } from '../services/api';

/**
 * Return type for useChat hook
 * @interface UseChatReturn
 */
export interface UseChatReturn {
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;
  sendMessage: (question: string) => Promise<void>;
  sendMessageWithContext: (question: string, selectedText: string) => Promise<void>;
  clearConversation: () => void;
  isOpen: boolean;
  setIsOpen: (open: boolean) => void;
}

/**
 * Custom hook for managing chat widget state and API calls
 * @returns {UseChatReturn} Chat state and methods
 */
export function useChat(): UseChatReturn {
  const [conversation, setConversation] = useState<Conversation>({
    id: `session-${Date.now()}`,
    messages: [],
    createdAt: new Date().toISOString(),
    updatedAt: new Date().toISOString(),
    isOpen: false,
  });

  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Load conversation from localStorage on mount
  useEffect(() => {
    if (typeof window !== 'undefined' && conversation.messages.length === 0) {
      try {
        const sessionId = conversation.id || `session-${Date.now()}`;
        const saved = localStorage.getItem(`chat_session_${sessionId}`);
        if (saved) {
          const parsed = JSON.parse(saved);
          setConversation(parsed);
        }
      } catch {
        // Silently fail if localStorage not available or corrupted
      }
    }
  }, []);

  // Save conversation to localStorage on change
  useEffect(() => {
    if (typeof window !== 'undefined' && conversation.messages.length > 0) {
      try {
        localStorage.setItem(
          `chat_session_${conversation.id}`,
          JSON.stringify(conversation)
        );
      } catch {
        // Silently fail if localStorage quota exceeded
      }
    }
  }, [conversation]);

  /**
   * Adds a user message and calls the API
   */
  const sendMessage = useCallback(
    async (question: string) => {
      // Validate input
      const trimmed = question.trim();
      if (!trimmed) {
        setError('Please enter a question');
        return;
      }

      if (trimmed.length > 2000) {
        setError('Question is too long (max 2000 characters)');
        return;
      }

      // Add user message
      const userMessage: ChatMessage = {
        id: `msg-${Date.now()}`,
        role: 'user',
        content: trimmed,
        timestamp: new Date().toISOString(),
        sources: [],
      };

      setConversation((prev) => ({
        ...prev,
        messages: [...prev.messages, userMessage],
        updatedAt: new Date().toISOString(),
      }));

      setError(null);
      setIsLoading(true);

      try {
        const response = await postChat(trimmed);

        // Add assistant message
        const assistantMessage: ChatMessage = {
          id: `msg-${Date.now()}`,
          role: 'assistant',
          content: response.answer,
          timestamp: new Date().toISOString(),
          sources: response.sources || [],
        };

        setConversation((prev) => ({
          ...prev,
          messages: [...prev.messages, assistantMessage],
          updatedAt: new Date().toISOString(),
        }));
      } catch (err) {
        const errorMsg =
          err instanceof APIError ? err.userMessage : 'Failed to send message';
        setError(errorMsg);

        // Add error message to conversation
        const errorMessage: ChatMessage = {
          id: `msg-${Date.now()}`,
          role: 'assistant',
          content: errorMsg,
          timestamp: new Date().toISOString(),
          sources: [],
          error: true,
        };

        setConversation((prev) => ({
          ...prev,
          messages: [...prev.messages, errorMessage],
          updatedAt: new Date().toISOString(),
        }));
      } finally {
        setIsLoading(false);
      }
    },
    []
  );

  /**
   * Sends a message with selected text context
   */
  const sendMessageWithContext = useCallback(
    async (question: string, selectedText: string) => {
      // Validate inputs
      const trimmedQuestion = question.trim();
      const trimmedText = selectedText.trim();

      if (!trimmedQuestion) {
        setError('Please enter a question');
        return;
      }

      if (trimmedQuestion.length > 2000) {
        setError('Question is too long (max 2000 characters)');
        return;
      }

      if (trimmedText.length > 5000) {
        setError('Selected text is too long (max 5000 characters)');
        return;
      }

      // Add user message with context indicator
      const userMessage: ChatMessage = {
        id: `msg-${Date.now()}`,
        role: 'user',
        content: trimmedQuestion,
        timestamp: new Date().toISOString(),
        sources: [],
      };

      setConversation((prev) => ({
        ...prev,
        messages: [...prev.messages, userMessage],
        updatedAt: new Date().toISOString(),
      }));

      setError(null);
      setIsLoading(true);

      try {
        const response = await postChatSelected(trimmedQuestion, trimmedText);

        // Add assistant message
        const assistantMessage: ChatMessage = {
          id: `msg-${Date.now()}`,
          role: 'assistant',
          content: response.answer,
          timestamp: new Date().toISOString(),
          sources: response.sources || [],
        };

        setConversation((prev) => ({
          ...prev,
          messages: [...prev.messages, assistantMessage],
          updatedAt: new Date().toISOString(),
        }));
      } catch (err) {
        const errorMsg =
          err instanceof APIError ? err.userMessage : 'Failed to send message';
        setError(errorMsg);

        // Add error message
        const errorMessage: ChatMessage = {
          id: `msg-${Date.now()}`,
          role: 'assistant',
          content: errorMsg,
          timestamp: new Date().toISOString(),
          sources: [],
          error: true,
        };

        setConversation((prev) => ({
          ...prev,
          messages: [...prev.messages, errorMessage],
          updatedAt: new Date().toISOString(),
        }));
      } finally {
        setIsLoading(false);
      }
    },
    []
  );

  /**
   * Clears the conversation history
   */
  const clearConversation = useCallback(() => {
    const emptyConversation: Conversation = {
      id: `session-${Date.now()}`,
      messages: [],
      createdAt: new Date().toISOString(),
      updatedAt: new Date().toISOString(),
      isOpen: false,
    };
    setConversation(emptyConversation);
    setError(null);

    // Clear localStorage
    if (typeof window !== 'undefined') {
      try {
        localStorage.removeItem(`chat_session_${conversation.id}`);
      } catch {
        // Silently fail
      }
    }
  }, [conversation.id]);

  return {
    messages: conversation.messages,
    isLoading,
    error,
    sendMessage,
    sendMessageWithContext,
    clearConversation,
    isOpen: conversation.isOpen,
    setIsOpen: (open: boolean) =>
      setConversation((prev) => ({ ...prev, isOpen: open })),
  };
}
