/**
 * Test Setup and Jest Configuration Helpers
 * Common test utilities and mocks for Chat Widget tests
 * @module __tests__/setup
 */

/**
 * Mock for API client functions
 */
export const mockApi = {
  postChat: jest.fn(),
  postChatSelected: jest.fn(),
};

/**
 * Mock for backend URL configuration
 */
export const mockGetBackendUrl = jest.fn(() => 'http://localhost:8000');

/**
 * Mock window.getSelection for text selection tests
 */
export function createMockSelection(text: string) {
  const range = document.createRange();
  return {
    toString: () => text,
    getRangeAt: () => range,
    rangeCount: 1,
    removeAllRanges: jest.fn(),
  };
}

/**
 * Render wrapper for components with providers
 */
import React from 'react';
import { render, RenderOptions } from '@testing-library/react';

export function renderWithProviders(
  ui: React.ReactElement,
  options?: Omit<RenderOptions, 'wrapper'>
) {
  return render(ui, { ...options });
}

export * from '@testing-library/react';
