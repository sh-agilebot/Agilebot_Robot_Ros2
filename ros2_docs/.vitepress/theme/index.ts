/* eslint-disable @agilebot/no-import-css -- 自定义主题 */

import { withAgilebotTheme } from '@agilebot/vitepress-preset/theme';

import './styles/overrides.scss';

export default withAgilebotTheme({
  // 谷歌统计
  gaTrackingId: 'G-BWM8VTXL9F'
});
