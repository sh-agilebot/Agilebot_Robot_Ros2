'use strict';

const serverApp = require('./shared/vitepress-export-pdf.a0128f3c.cjs');
require('node:path');
require('node:process');
require('node:module');
require('vitepress');
require('debug');
require('hash-sum');
require('@condorhero/vuepress-plugin-export-pdf-core');

const defineUserConfig = (config) => config;

exports.HOST = serverApp.HOST;
exports.PORT = serverApp.PORT;
exports.moduleRequire = serverApp.moduleRequire;
exports.serverApp = serverApp.serverApp;
exports.defineUserConfig = defineUserConfig;
