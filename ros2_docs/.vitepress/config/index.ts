import path from 'node:path';

import { dirname } from 'dirname-filename-esm';
import { withAgilebotConfig } from '@agilebot/vitepress-preset/config';
import { withMermaid } from 'vitepress-plugin-mermaid';
import taskLists from 'markdown-it-task-lists';

const __dirname = dirname(import.meta);

const DOC_ROOT = path.join(__dirname, '../..');
const VP_ROOT = path.join(DOC_ROOT, '.vitepress');
const BASE_URL = process.env.NODE_ENV === 'production' ? '/docs/ros/' : '/';
const DEV_CENTER_URL = process.env.DEV_CENTER_HOST
  ? `http://${process.env.DEV_CENTER_HOST}:8080`
  : 'https://dev.sh-agilebot.com';

const baseConfig = await withAgilebotConfig({
  docRoot: DOC_ROOT,
  vpRoot: VP_ROOT,
  base: BASE_URL,
  title: '捷勃特机器人 ROS2 说明书',
  description: '捷勃特机器人 ROS2 说明书',
  locales: {
    zh: {
      label: '中文',
      lang: 'zh',
      title: '捷勃特机器人 ROS2 说明书',
      description: '捷勃特机器人 ROS2 说明书',
      themeConfig: {
        nav: [
          {
            text: '开发者中心',
            link: DEV_CENTER_URL
          }
        ]
      }
    },
    en: {
      label: 'English',
      lang: 'en',
      title: 'Agilebot ROS2 Documentation',
      description: 'Agilebot ROS2 Documentation',
      themeConfig: {
        nav: [
          {
            text: 'Developer Center',
            link: DEV_CENTER_URL
          }
        ]
      }
    }
  },
  pluginConfig: {
    sidebar: {
      // 默认折叠
      collapsed: true,
      // 折叠深度
      collapseDepth: 2
    }
  },
  markdown: {
    config: (md) => {
      md.use(taskLists); // 启用任务列表
    }
  },
  vite: {
    optimizeDeps: {
      include: ['mermaid']
    }
  }
});

const config = withMermaid({
  ...baseConfig,
  mermaid: {
    // refer https://mermaid.js.org/config/setup/modules/mermaidAPI.html#mermaidapi-configuration-defaults for options
  },
  // optionally set additional config for plugin itself with MermaidPluginConfig
  mermaidPlugin: {
    class: 'mermaid my-class' // set additional css classes for parent container
  }
});

export default config;
