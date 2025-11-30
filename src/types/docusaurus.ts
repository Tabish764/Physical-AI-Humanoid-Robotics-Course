/**
 * Docusaurus-specific TypeScript types and interfaces
 * Type definitions for Docusaurus hooks and configuration
 * @module types/docusaurus
 */

/**
 * Docusaurus site configuration
 * @interface DocusaurusSiteConfig
 */
export interface DocusaurusSiteConfig {
  /** Base URL of the site */
  baseUrl: string;

  /** Site title */
  title: string;

  /** Site URL */
  url: string;

  /** Custom fields from docusaurus.config.js */
  customFields?: Record<string, unknown>;

  /** Favicon path */
  favicon?: string;

  /** Organization configuration */
  organizationName?: string;

  /** Project name */
  projectName?: string;
}

/**
 * Docusaurus context object returned by useDocusaurusContext()
 * @interface DocusaurusContext
 */
export interface DocusaurusContext {
  /** Site configuration */
  siteConfig: DocusaurusSiteConfig;

  /** Whether in SSR mode */
  isClient: boolean;
}

/**
 * Type for useDocusaurusContext hook
 * @type {() => DocusaurusContext}
 */
export type UseDocusaurusContextType = () => DocusaurusContext;

/**
 * Type for useBaseUrl hook
 * @type {(path: string, options?: {forcePrependBaseUrl?: boolean}) => string}
 */
export type UseBaseUrlType = (
  path: string,
  options?: { forcePrependBaseUrl?: boolean }
) => string;

/**
 * Type for Link component from Docusaurus
 */
export interface DocusaurusLinkProps {
  to?: string;
  href?: string;
  className?: string;
  children?: React.ReactNode;
  onClick?: (e: React.MouseEvent) => void;
}

/**
 * Type for useLocation hook
 * @interface Location
 */
export interface Location {
  pathname: string;
  search: string;
  hash: string;
}
