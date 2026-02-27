'use strict';

const vuepressPluginExportPdfCore = require('@condorhero/vuepress-plugin-export-pdf-core');
const serverApp = require('../shared/vitepress-export-pdf.a0128f3c.cjs');
require('node:path');
require('node:process');
require('node:module');
require('vitepress');
require('debug');
require('hash-sum');

function registerCommands(program) {
  program.command("export [sourceDir]", "Export current VitePress site to a PDF file(default: docs)").allowUnknownOptions().option("-c, --config <config>", "Set path to config file").option("--outFile <outFile>", "Name of output file").option("--outDir <outDir>", "Directory of output files").option("--pdfOutlines <pdfOutlines>", "Keep PDF outlines/bookmarks").option("--urlOrigin <urlOrigin>", "Change the origin of the print url(Option displayHeaderFooter of pdfOptions is true)").option("--debug", "Enable debug mode").action(serverApp.serverApp);
  program.command("info", "Display environment information").action(() => {
    vuepressPluginExportPdfCore.systemInfo(["vitepress"]);
  });
}
vuepressPluginExportPdfCore.runCli("press-export-pdf")(registerCommands);

exports.registerCommands = registerCommands;
