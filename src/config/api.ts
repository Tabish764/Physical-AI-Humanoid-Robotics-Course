/**
 * API Configuration Module
 * Handles environment-based backend URL configuration
 * @module config/api
 */

/**
 * Gets the backend API base URL from environment or Docusaurus config
 * Priority order:
 * 1. REACT_APP_BACKEND_URL environment variable
 * 2. process.env.REACT_APP_BACKEND_URL (build-time)
 * 3. Docusaurus customFields.backendUrl (from docusaurus.config.js)
 * 4. Fallback to localhost:8000 (development)
 *
 * @returns {string} Backend URL with protocol and host (e.g., http://localhost:8000)
 * @throws {Error} If running in browser and backend URL not configured
 */
export function getBackendUrl(): string {
  // Check runtime environment variable (client-side)
  if (typeof window !== 'undefined') {
    const envUrl = (window as any).REACT_APP_BACKEND_URL;
    if (envUrl) {
      return validateAndNormalizeUrl(envUrl);
    }
  }

  // Check build-time environment variable
  if (typeof process !== 'undefined' && process.env?.REACT_APP_BACKEND_URL) {
    return validateAndNormalizeUrl(process.env.REACT_APP_BACKEND_URL);
  }

  // Try to get from Docusaurus config (if available)
  try {
    const docusaurusConfig = (window as any).__DOCUSAURUS_CONFIG_;
    if (docusaurusConfig?.customFields?.backendUrl) {
      return validateAndNormalizeUrl(docusaurusConfig.customFields.backendUrl);
    }
  } catch {
    // Silently ignore if Docusaurus config not available
  }

  // Development fallback
  if (typeof window !== 'undefined' && window.location.hostname === 'localhost') {
    return 'http://localhost:8000';
  }

  throw new Error(
    'Backend URL not configured. Set REACT_APP_BACKEND_URL environment variable or configure backendUrl in docusaurus.config.js customFields'
  );
}

/**
 * Validates and normalizes the backend URL
 * Ensures proper protocol and removes trailing slashes
 *
 * @param {string} url - The URL to validate
 * @returns {string} Normalized URL
 * @throws {Error} If URL format is invalid
 */
function validateAndNormalizeUrl(url: string): string {
  if (!url || typeof url !== 'string') {
    throw new Error('Backend URL must be a non-empty string');
  }

  let normalizedUrl = url.trim();

  // Add protocol if missing
  if (!normalizedUrl.startsWith('http://') && !normalizedUrl.startsWith('https://')) {
    normalizedUrl = `http://${normalizedUrl}`;
  }

  // Validate URL format
  try {
    const urlObj = new URL(normalizedUrl);
    // Remove trailing slash
    return urlObj.origin;
  } catch {
    throw new Error(`Invalid backend URL format: ${url}`);
  }
}

/**
 * Checks if the backend is accessible
 * Useful for health checks or connection verification
 *
 * @returns {Promise<boolean>} True if backend is reachable
 */
export async function isBackendAccessible(): Promise<boolean> {
  try {
    const response = await fetch(`${getBackendUrl()}/api/health`, {
      method: 'GET',
      signal: AbortSignal.timeout(5000), // 5 second timeout
    });
    return response.ok;
  } catch {
    return false;
  }
}

/**
 * Gets the full API endpoint URL
 * @param {string} path - API endpoint path (e.g., '/api/chat')
 * @returns {string} Full URL to the endpoint
 */
export function getApiUrl(path: string): string {
  const baseUrl = getBackendUrl();
  const cleanPath = path.startsWith('/') ? path : `/${path}`;
  return `${baseUrl}${cleanPath}`;
}
