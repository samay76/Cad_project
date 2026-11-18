/// <reference types="vite/client" />

declare module 'occt-import-js' {
  const occtimportjs: (params?: any) => Promise<any>;
  export default occtimportjs;
}
