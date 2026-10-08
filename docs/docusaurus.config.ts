import { createRequire } from "node:module";
import type * as Preset from "@docusaurus/preset-classic";
import type { Config } from "@docusaurus/types";
import { themes as prismThemes } from "prism-react-renderer";
import rehypeKatex from "rehype-katex";
import remarkMath from "remark-math";

const require = createRequire(import.meta.url);

// Served by the bldc-sim binary at /docs (D-009), so baseUrl is /docs/ and docs live at its root.
const config: Config = {
  title: "BLDC Motor Simulator",
  tagline: "Learn, reproduce and play with brushless PM motors",
  favicon: "img/logo.svg",
  url: "http://localhost:8787",
  baseUrl: "/docs/",
  onBrokenLinks: "throw",
  onBrokenAnchors: "throw",
  markdown: { mermaid: true, hooks: { onBrokenMarkdownLinks: "throw" } },
  i18n: { defaultLocale: "en", locales: ["en"] },
  future: { v4: true, faster: true },

  presets: [
    [
      "classic",
      {
        docs: {
          routeBasePath: "/",
          sidebarPath: "./sidebars.ts",
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex],
        },
        blog: false,
        theme: { customCss: "./src/css/custom.css" },
      } satisfies Preset.Options,
    ],
  ],

  // KaTeX CSS bundled locally (same version rehype-katex renders with), so math works offline.
  clientModules: [require.resolve("katex/dist/katex.min.css")],

  themes: [
    "@docusaurus/theme-mermaid",
    [
      "@easyops-cn/docusaurus-search-local",
      { hashed: true, docsRouteBasePath: "/", indexBlog: false },
    ],
  ],

  plugins: [
    [
      "docusaurus-plugin-llms",
      { generateLLMsTxt: true, generateLLMsFullTxt: true, docsDir: "docs" },
    ],
  ],

  themeConfig: {
    colorMode: { respectPrefersColorScheme: true },
    navbar: {
      title: "BLDC Motor Simulator",
      logo: { alt: "BLDC simulator logo", src: "img/logo.svg" },
      items: [{ type: "docSidebar", sidebarId: "docs", position: "left", label: "Docs" }],
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ["rust", "toml", "yaml"],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
