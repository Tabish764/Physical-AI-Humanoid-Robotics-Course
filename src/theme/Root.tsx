import React from 'react';
import type {FC, ReactNode} from 'react';
import {ChatWidget} from '../components/ChatWidget';

/**
 * Root theme component
 * 
 * Swizzles the default Docusaurus root to inject the ChatWidget
 * globally across all pages. The ChatWidget persists state during
 * navigation and is available on every page of the documentation.
 * 
 * @param props - Standard Docusaurus Root component props
 * @returns JSX with ChatWidget mounted at root level
 */
const Root: FC<{children: ReactNode}> = ({children}) => {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
};

export default Root;
