import mdx from '@astrojs/mdx';
import sitemap from '@astrojs/sitemap';
import tailwindcss from '@tailwindcss/vite';
import { defineConfig } from 'astro/config';
import siteConfig from './src/data/site-config';

import { unified } from '@astrojs/markdown-remark';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

export default defineConfig({
    site: siteConfig.website,

    vite: {
        plugins: [tailwindcss()]
    },

    integrations: [
        mdx(),
        sitemap()
    ],

    markdown: {
        // 使用 unified Markdown 处理器
        processor: unified({
            remarkPlugins: [
                remarkMath
            ],

            rehypePlugins: [
                rehypeKatex
            ]
        }),

        // Shiki 代码高亮
        syntaxHighlight: 'shiki',

        shikiConfig: {
            theme: 'github-dark',
            wrap: true
        }
    }
});