import avatar from '../assets/images/avatar.jpg';
import hero from '../assets/images/cleanrobot.png';
import type { SiteConfig } from '../types';

const siteConfig: SiteConfig = {
    website: 'https://wangfei-tech.github.io',
    avatar: {
        src: avatar,
        alt: 'Ethan Donovan'
    },
    title: 'Robotics Engineering',
    subtitle: 'ROS 2 | SLAM | Navigation | C++',
    description: '个人机器人技术博客，记录 ROS 2、SLAM、自主导航、覆盖路径规划与机器人运动控制相关的算法研究和工程实践',
    image: {
        src: '/dante-preview.jpg',
        alt: 'Dante - Astro.js and Tailwind CSS theme'
    },
    headerNavLinks: [
        {
            text: 'Home',
            href: '/'
        },
        {
            text: 'Projects',
            href: '/projects'
        },
        {
            text: 'Blog',
            href: '/blog'
        },
        {
            text: 'Tags',
            href: '/tags'
        }
    ],
    footerNavLinks: [
        {
            text: 'About',
            href: '/about'
        },
        {
            text: 'Contact',
            href: '/contact'
        },
        {
            text: 'Terms',
            href: '/terms'
        },
        {
            text: 'Download theme',
            href: 'https://github.com/JustGoodUI/dante-astro-theme'
        }
    ],
    socialLinks: [
        {
            text: 'Dribbble',
            href: 'https://dribbble.com/'
        },
        {
            text: 'Instagram',
            href: 'https://instagram.com/'
        },
        {
            text: 'X/Twitter',
            href: 'https://twitter.com/'
        }
    ],
    hero: {
        title: 'Building Intelligent Mobile Robots',
        text: "专注于移动机器人自主导航、定位建图与运动控制。主要技术方向包括 **ROS 2、SLAM、Nav2、Coverage Path Planning 和 C++**。这个网站用于记录我的机器人算法项目、工程问题分析以及实际机器人测试成果。欢迎浏览我的项目与技术文章。",
        image: {
            src: hero,
            alt: 'A person sitting at a desk in front of a computer'
        },
        actions: [
            {
                text: 'Explore My Projects',
                href: '/projects'
            }
        ]
    },
    subscribe: {
        enabled: true,
        title: 'Subscribe to Dante Newsletter',
        text: 'One update per week. All the latest posts directly in your inbox.',
        form: {
            action: '#'
        }
    },
    postsPerPage: 8,
    projectsPerPage: 8
};

export default siteConfig;
