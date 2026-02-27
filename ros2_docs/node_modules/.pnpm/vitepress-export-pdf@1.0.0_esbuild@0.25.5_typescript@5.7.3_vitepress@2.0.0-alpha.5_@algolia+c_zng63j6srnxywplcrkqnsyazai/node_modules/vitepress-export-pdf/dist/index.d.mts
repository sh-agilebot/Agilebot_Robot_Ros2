import { Page, LaunchOptions, PDFOptions, CommandOptions } from '@condorhero/vuepress-plugin-export-pdf-core';

type PageType = Omit<Page, 'title'>;
type UserSorter = (a: PageType, b: PageType) => number;
/**
 * defined user config
 */
interface UserConfig {
    sorter?: UserSorter;
    puppeteerLaunchOptions?: LaunchOptions;
    routePatterns?: string[];
    pdfOptions?: PDFOptions;
    outFile?: string;
    outDir?: string;
    pdfOutlines?: boolean;
    urlOrigin?: string;
    outlineContainerSelector?: string;
}

declare const PORT = 16762;
declare const HOST = "localhost";
declare const moduleRequire: NodeRequire;
declare function serverApp(dir?: string, commandOptions?: CommandOptions): Promise<void>;

/**
 * defineUserConfig is a function that defines the user config.
 * @param config - UserConfig
 * @returns config - UserConfig
 */
declare const defineUserConfig: (config: UserConfig) => UserConfig;

export { HOST, PORT, type PageType, type UserConfig, type UserSorter, defineUserConfig, moduleRequire, serverApp };
