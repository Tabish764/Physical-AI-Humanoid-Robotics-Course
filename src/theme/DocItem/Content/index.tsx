import React from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type {WrapperProps} from '@docusaurus/types';


/**
 * Swizzled DocItem/Content component
 * 
 * Injects the UrduButton at the top of each documentation page.
 * The button appears above the main content area.
 */
export default function ContentWrapper(props: WrapperProps<typeof ContentType>): JSX.Element {
  return (
    <>
      <div style={{display: 'flex', justifyContent: 'flex-end', marginBottom: '1rem'}}>
    
      </div>
      <Content {...props} />
    </>
  );
}

