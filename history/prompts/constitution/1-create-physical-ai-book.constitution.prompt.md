---
id: 1
title: Create Physical AI Book Constitution
stage: constitution
date: 2025-12-14
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: Claude
command: /sp.constitution
labels: [constitution, physical-ai, robotics, book]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```
Project: Book on Physical AI & Humanoid Robotics
(using Docusaurus + Claude Code + Spec-Kit Plus + Integrated RAG Chatbot)

Vision:
- Create the definitive beginner-friendly guide to Physical AI, Humanoid Robotics, and Embodied Intelligence.
- Teach students how to bridge digital intelligence (AI models) with physical systems (robots).
- Provide hands-on, simulation-driven robotics learning using ROS 2, Gazebo, Unity, and NVIDIA Isaac.

Core principles:
- Hands-on, implementation-first teaching philosophy
- Clarity for beginners; depth for intermediate learners
- Technical accuracy across robotics, physics simulation, and AI systems
- Practical robotics workflows: ROS 2 → Gazebo/Unity → Isaac → VLA systems
- Documentation quality aligned with industry-grade engineering docs

Key standards:
- All technical explanations must be validated against official ROS 2, Gazebo, Unity, and NVIDIA Isaac documentation
- Code samples must be runnable, tested, and minimal
- Diagrams for architecture, pipelines, and robot systems required in every module
- Tone: accessible, encouraging, and focused on real-world skills
- Consistent structure across chapters: Concept → Architecture → Code → Simulation → Practice Task
- Chatbot RAG answers must rely strictly on indexed book content

Constraints:
- Format: Docusaurus (MDX)
- Deployment: GitHub Pages + embedded RAG chatbot
- RAG stack: FastAPI, Neon Serverless Postgres, Qdrant Cloud, OpenAI Agents/ChatKit SDK
- All chapters must be chunkable for retrieval (≤1500 tokens per chunk)
- Book must support code-block execution within Claude Code for iterative drafting

Success criteria:
- Complete 4-module book with hands-on examples and simulations
- Fully functional RAG chatbot that answers user questions based on book content only
- Accurate robotic simulations demonstrated across ROS 2, Gazebo/Unity, and Isaac
- Learners can build a final capstone: an autonomous humanoid robot executing natural-language commands
- Documentation passes clarity, accuracy, and usability review
```

## Response snapshot

Constitution file has been successfully created at `.specify/memory/constitution.md` with the following content:

# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### I. Hands-on, Implementation-First Teaching Philosophy
All content must prioritize practical, hands-on learning over purely theoretical explanations. Every concept must be accompanied by runnable code examples, simulations, or practical exercises that students can execute and modify.

### II. Clarity for Beginners, Depth for Intermediate Learners
Content must be accessible to beginners without sacrificing technical depth for intermediate learners. All explanations must start with fundamental concepts before advancing to complex topics, with clear pathways for different skill levels.

### III. Technical Accuracy Across Robotics Systems
All technical explanations must be validated against official ROS 2, Gazebo, Unity, and NVIDIA Isaac documentation. Code samples must be runnable, tested, and minimal. Diagrams for architecture, pipelines, and robot systems required in every module.

### IV. Practical Robotics Workflows
Content must demonstrate real-world robotics workflows: ROS 2 → Gazebo/Unity → Isaac → VLA systems. Each chapter must follow a consistent structure: Concept → Architecture → Code → Simulation → Practice Task.

### V. Documentation Quality Standards
All content must meet industry-grade engineering documentation standards. Tone must be accessible, encouraging, and focused on real-world skills. Content must be structured consistently across chapters with clear learning objectives.

### VI. RAG Chatbot Content Integrity
The integrated RAG chatbot must rely strictly on indexed book content for answers. No external information sources should be used in responses. All chatbot answers must be traceable to specific sections of the book content.

## Technology Stack Requirements

All book content and associated tools must adhere to the following technology stack:
- Format: Docusaurus (MDX)
- Deployment: GitHub Pages + embedded RAG chatbot
- RAG stack: FastAPI, Neon Serverless Postgres, Qdrant Cloud, OpenAI Agents/ChatKit SDK
- All chapters must be chunkable for retrieval (≤1500 tokens per chunk)
- Book must support code-block execution within Claude Code for iterative drafting

## Development Workflow and Quality Standards

All content development must follow these standards:
- Content must be validated against official ROS 2, Gazebo, Unity, and NVIDIA Isaac documentation
- Code samples must be tested and runnable in simulation environments
- Each chapter must include diagrams for architecture, pipelines, and robot systems
- Content must follow the consistent structure: Concept → Architecture → Code → Simulation → Practice Task
- All technical explanations must be accurate and up-to-date with current robotics frameworks

## Governance

This constitution governs all aspects of the Physical AI & Humanoid Robotics book development. All contributors must adhere to these principles and standards. Amendments to this constitution require documentation of changes, approval from project maintainers, and a migration plan for existing content. All pull requests and reviews must verify compliance with these principles before approval.

**Version**: 1.0.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14

## Outcome

- ✅ Impact: New constitution created for Physical AI & Humanoid Robotics book project
- 🧪 Tests: none
- 📁 Files: 1 file created (.specify/memory/constitution.md)
- 🔁 Next prompts: Continue with spec, plan, and tasks creation
- 🧠 Reflection: The constitution captures all key principles and constraints from the project description

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating the feature specification