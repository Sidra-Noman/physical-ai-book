import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics Book',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Module 1: The Robotic Nervous System (ROS 2)',
          link: {type: 'doc', id: 'module-1-robotic-nervous-system/index'},
          items: [
            'module-1-robotic-nervous-system/ros2-concepts',
            'module-1-robotic-nervous-system/ai-ros-integration',
            'module-1-robotic-nervous-system/urdf-modeling'
          ]
        },
        {
          type: 'category',
          label: 'Module 2: The Digital Twin (Gazebo & Unity)',
          link: {type: 'doc', id: 'module-2-digital-twin/index'},
          items: [
            'module-2-digital-twin/gazebo-simulation',
            'module-2-digital-twin/unity-integration',
            'module-2-digital-twin/sensor-simulation'
          ]
        },
        {
          type: 'category',
          label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
          link: {type: 'doc', id: 'module-3-ai-robot-brain/index'},
          items: [
            'module-3-ai-robot-brain/nvidia-isaac-sim',
            'module-3-ai-robot-brain/isaac-ros-pipelines'
          ]
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA)',
          link: {type: 'doc', id: 'module-4-vla/index'},
          items: [
            'module-4-vla/voice-action-pipelines',
            'module-4-vla/llm-cognitive-planning'
          ]
        },
        {
          type: 'category',
          label: 'Module 5: The Autonomous Humanoid Capstone',
          link: {type: 'doc', id: 'module-5-capstone/index'},
          items: [
            'module-5-capstone/capstone-architecture',
            'module-5-capstone/autonomous-humanoid'
          ]
        }
      ]
    }
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
