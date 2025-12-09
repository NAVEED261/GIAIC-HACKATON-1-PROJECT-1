---
feature: Physical AI Textbook Structure
version: 1.0.0
status: Draft
created: 2025-12-09
updated: 2025-12-09
author: System
---

# Tasks: Physical AI Textbook Structure

## Overview

This document breaks down the implementation of the Physical AI Textbook Structure into actionable, testable tasks organized by user story. Each phase represents an independently deployable increment.

**Related Documents:**
- Spec: specs/1-physical-ai-textbook-structure/spec.md
- Plan: specs/1-physical-ai-textbook-structure/plan.md
- Data Model: specs/1-physical-ai-textbook-structure/data-model.md
- Research: specs/1-physical-ai-textbook-structure/research/research.md

**Technology Stack:**
- Docusaurus v3 (static site generator)
- React 18 + TypeScript (custom components)
- Algolia DocSearch (content search)
- Fuse.js (glossary fuzzy search)
- React Flow + dagre (prerequisite graph visualization)
- Node.js 18+ (build runtime)

## Implementation Strategy

**MVP Scope:** User Story 1 only (Navigate Complete Course Structure)
- Delivers homepage dashboard, sidebar navigation, and basic directory structure
- Allows students to see course organization immediately
- Enables incremental content authoring

**Incremental Delivery:**
1. Deploy US1 → Students can navigate empty structure
2. Deploy US2 → Students can set up environment
3. Deploy US3-5 → Enhanced features (assessments, references)

**Parallel Opportunities:**
- Custom components can be developed in parallel (US1, US2)
- Reference materials can be created independently (US5)
- Templates and schemas are independent tasks

## Dependencies

### User Story Completion Order

```
Phase 1 (Setup) → Phase 2 (Foundational)
                    ↓
        ┌───────────┴───────────┐
        ↓                       ↓
    Phase 3 (US1)           Phase 4 (US2)
        ↓                       ↓
        └───────────┬───────────┘
                    ↓
        ┌───────────┴───────────┐
        ↓                       ↓
    Phase 5 (US3)           Phase 6 (US4)
        ↓                       ↓
        └───────────┬───────────┘
                    ↓
                Phase 7 (US5)
                    ↓
                Phase 8 (Polish)
```

**Key Dependencies:**
- US1 must complete before US3 (module navigation requires sidebar structure)
- US2 can run in parallel with US1 (independent features)
- US4 depends on US3 (assessments reference modules)
- US5 is independent (can run anytime after Phase 2)

### Parallel Execution Examples

**Phase 3 (US1) - 5 tasks can run in parallel:**
```bash
# Parallel group 1: Independent component files
T012, T013, T014  # ModuleCard, QuickLinksPanel, homepage layout

# Parallel group 2: Independent directory creation
T015, T016  # Module directories
```

**Phase 4 (US2) - 3 tasks can run in parallel:**
```bash
# All hardware setup guides are independent
T021, T022, T023  # Workstation, Edge Kit, Cloud setup
```

## Task List

### Phase 1: Setup (Project Initialization)

**Goal:** Scaffold Docusaurus project and establish foundational configuration

**Acceptance Criteria:**
- [ ] Docusaurus project builds successfully (`npm run build`)
- [ ] Development server runs (`npm start`)
- [ ] Git repository is initialized with proper .gitignore
- [ ] Package.json has all required dependencies

**Tasks:**

- [X] T001 Initialize Docusaurus v3 project using `npx create-docusaurus@latest physical-ai-textbook classic --typescript` in project root
- [X] T002 Install additional dependencies: `npm install fuse.js reactflow dagre @docusaurus/plugin-ideal-image`
- [X] T003 [P] Configure docusaurus.config.ts with site metadata (title: "Physical AI Textbook", tagline, url, baseUrl, GitHub repo)
- [X] T004 [P] Create .gitignore file including node_modules/, .docusaurus/, build/, .env
- [X] T005 [P] Initialize Git repository with initial commit of scaffolded structure

---

### Phase 2: Foundational (Blocking Prerequisites)

**Goal:** Create shared infrastructure needed by all user stories

**Acceptance Criteria:**
- [ ] Directory structure matches plan.md specification
- [ ] Sidebar configuration supports nested categories
- [ ] Metadata schema is documented and validated
- [ ] Build pipeline passes all quality gates

**Tasks:**

- [X] T006 Create docs/ directory structure with subdirectories: intro/, setup/, module-1-ros2/, module-2-digital-twin/, module-3-isaac/, module-4-vla/, capstone/, assessments/, reference/, instructors/
- [X] T007 Create static/ directory structure with subdirectories: data/, img/course-structure/, img/modules/
- [X] T008 Create src/components/ directory for custom React components
- [X] T009 Create src/css/custom.css with CSS variables for module colors (Module 1: #3578e5, Module 2: #8e44ad, Module 3: #e67e22, Module 4: #27ae60)
- [X] T010 [P] Configure sidebars.ts with nested category structure per plan.md (Introduction, Setup, Modules 1-4, Capstone, Assessments, Reference, Instructors)
- [X] T011 [P] Create static/data/modules.json file with 4 module metadata entries (id, title, moduleNumber, weekRange, estimatedHours, learningOutcomes, capstoneIntegration, color)

---

### Phase 3: User Story 1 - Navigate Complete Course Structure (P1)

**Goal:** Enable students to see complete 13-week course structure and navigate by week/module/topic

**User Story:** As an industry practitioner learning Physical AI, I need to see the complete 13-week course structure organized by modules and weeks, so that I can plan my learning journey and understand prerequisite relationships between topics.

**Independent Test Criteria:**
- [ ] Homepage displays 4 module cards with correct week ranges
- [ ] Sidebar shows all 13 weeks organized by modules
- [ ] Each module category is collapsible
- [ ] Quick links sidebar is visible and functional
- [ ] Navigation requires maximum 2 clicks from homepage to any week

**Tasks:**

- [X] T012 [P] [US1] Create ModuleCard component in src/components/ModuleCard.tsx with props interface (moduleNumber, title, weekRange, learningOutcomes, estimatedHours, link, color)
- [X] T013 [P] [US1] Create QuickLinksPanel component in src/components/QuickLinksPanel.tsx with props interface (links array with label, url, icon)
- [X] T014 [P] [US1] Create custom homepage in src/pages/index.tsx with dashboard layout (hero section + 4 module cards in 2x2 grid + quick links sidebar)
- [X] T015 [P] [US1] Create module index files: docs/module-1-ros2/index.md, docs/module-2-digital-twin/index.md, docs/module-3-isaac/index.md, docs/module-4-vla/index.md with frontmatter (title, description, sidebar_position)
- [X] T016 [P] [US1] Create week-based subdirectories under each module: module-1-ros2/week-3-basics/, week-4-communication/, week-5-integration/ (similarly for modules 2-4)
- [X] T017 [US1] Update sidebars.ts to add nested week categories under each module with collapsed: true for modules
- [X] T018 [US1] Create docs/intro/index.md with course overview, structure diagram, and learning outcomes (frontmatter: week: 1, module: 0, estimated_time: 2)
- [X] T019 [US1] Create placeholder chapter files with frontmatter for Week 3-5 in module-1-ros2/ (ros2-fundamentals.md, nodes-topics.md, services.md, actions.md, multi-node-systems.md)
- [X] T020 [US1] Update homepage quick links to reference Hardware Setup (/docs/setup), Assessments (/docs/assessments), Glossary (/docs/reference/glossary)

---

### Phase 4: User Story 2 - Access Foundational Setup Documentation (P1)

**Goal:** Provide immediate access to hardware setup guides and glossary search

**User Story:** As an industry practitioner, I need immediate access to hardware setup guides, environment configuration, and glossary references before starting course content, so that I can prepare my development environment and understand terminology.

**Independent Test Criteria:**
- [ ] All 3 hardware setup guides are accessible and complete
- [ ] Glossary search component provides instant lookup (<2s response)
- [ ] Software installation guides include version specifications
- [ ] Hardware comparison table is present

**Tasks:**

- [X] T021 [P] [US2] Create docs/setup/hardware-digital-twin.md with hardware requirements (RTX 3060+, Ubuntu 22.04, 32GB RAM), cost estimate, installation steps, verification commands
- [X] T022 [P] [US2] Create docs/setup/hardware-edge-kit.md with Jetson Orin Nano specifications, setup steps, limitations, cost estimate
- [X] T023 [P] [US2] Create docs/setup/hardware-cloud.md with AWS/Azure setup instructions, cost estimates, cloud-specific limitations
- [X] T024 [P] [US2] Create docs/setup/software-ros2.md with ROS 2 Humble installation steps for Ubuntu 22.04, environment setup, verification (`ros2 --version`)
- [X] T025 [P] [US2] Create docs/setup/software-isaac-sim.md with Isaac Sim installation for workstation and cloud, GPU driver requirements, Omniverse setup
- [X] T026 [US2] Create docs/setup/index.md with hardware comparison table (3 options × features/cost/limitations) and decision tree
- [X] T027 [US2] Create GlossarySearch component in src/components/GlossarySearch.tsx using Fuse.js (fuzzy matching threshold: 0.3, search keys: term, definition)
- [X] T028 [US2] Create static/data/glossary.json with initial 100+ robotics terms (term, aliases, definition, relatedChapters, externalLinks)
- [X] T029 [US2] Create docs/reference/glossary.md embedding GlossarySearch component and rendering all terms alphabetically
- [X] T030 [US2] Add glossary search widget to navbar or sidebar in docusaurus.config.ts theme configuration

---

### Phase 5: User Story 3 - Follow Module-Based Learning Path (P2)

**Goal:** Enable sequential module progression with clear learning outcomes and capstone integration

**User Story:** As an industry practitioner, I need to progress through each module (ROS 2 → Digital Twin → Isaac → VLA) sequentially with clear module objectives and capstone integration points, so that I understand how each module builds toward the final autonomous humanoid project.

**Independent Test Criteria:**
- [ ] Each module index states learning outcomes
- [ ] Capstone integration points are documented per module
- [ ] Module progression is clear (Week 3-5 → Week 6-7 → Week 8-10 → Week 11-13)
- [ ] Students can estimate time commitment per module

**Tasks:**

- [X] T031 [P] [US3] Update docs/module-1-ros2/index.md with complete learning outcomes (3-5 outcomes), week range (3-5), estimated hours (30), capstone integration (communication layer)
- [X] T032 [P] [US3] Update docs/module-2-digital-twin/index.md with learning outcomes, week range (6-7), estimated hours (20), capstone integration (simulation testing)
- [X] T033 [P] [US3] Update docs/module-3-isaac/index.md with learning outcomes, week range (8-10), estimated hours (30), capstone integration (perception pipeline)
- [X] T034 [P] [US3] Update docs/module-4-vla/index.md with learning outcomes, week range (11-13), estimated hours (30), capstone integration (VLA decision making)
- [X] T035 [US3] Create docs/capstone/index.md with 5-component architecture diagram (voice → plan → navigate → perceive → manipulate) and integration overview
- [X] T036 [P] [US3] Create static/img/course-structure/course-timeline.svg showing 13-week progression with module boundaries and capstone milestones
- [X] T037 [P] [US3] Create static/img/course-structure/capstone-architecture.svg showing 5-component system with data flow arrows
- [X] T038 [US3] Add estimated_time totals to each module index page and validate sum is 130-156 hours (SC-006 compliance)

---

### Phase 6: User Story 4 - Access Assessment and Project Guidelines (P2)

**Goal:** Provide clear assessment criteria and capstone project requirements

**User Story:** As an industry practitioner, I need to access assessment criteria, project rubrics, and capstone requirements throughout the course, so that I can self-assess my progress and understand evaluation expectations.

**Independent Test Criteria:**
- [ ] All 4 assessment guides are accessible
- [ ] Each assessment has a 3-level rubric (needs improvement, proficient, excellent)
- [ ] Capstone architecture is clearly documented with 5 components
- [ ] Self-assessment checklists are present

**Tasks:**

- [X] T039 [P] [US4] Create docs/assessments/ros2-package-project.md with requirements (2+ nodes, topic communication, unit tests), rubric (3 criteria × 3 levels), submission guidelines
- [X] T040 [P] [US4] Create docs/assessments/gazebo-simulation-project.md with robot model requirements, simulation scenarios, rubric, submission guidelines
- [X] T041 [P] [US4] Create docs/assessments/isaac-perception-project.md with perception pipeline requirements (camera, lidar, object detection), rubric, submission guidelines
- [X] T042 [US4] Create docs/assessments/capstone-project.md with complete 5-component architecture requirements, integration checklist, evaluation rubric (6+ criteria)
- [X] T043 [US4] Create docs/assessments/index.md with assessment timeline (Week 5: ROS 2, Week 7: Gazebo, Week 10: Isaac, Week 13: Capstone) and overview
- [X] T044 [P] [US4] Create docs/capstone/voice-interface.md detailing voice component requirements (speech recognition, NLU, command parsing)
- [X] T045 [P] [US4] Create docs/capstone/task-planning.md detailing planning component (task decomposition, PDDL/BT, goal management)
- [X] T046 [P] [US4] Create docs/capstone/navigation.md detailing navigation component (SLAM, path planning, obstacle avoidance)
- [X] T047 [P] [US4] Create docs/capstone/perception.md detailing perception component (camera, lidar, object detection, pose estimation)
- [X] T048 [P] [US4] Create docs/capstone/manipulation.md detailing manipulation component (inverse kinematics, grasp planning, force control)

---

### Phase 7: User Story 5 - Reference Quick Guides and Troubleshooting (P3)

**Goal:** Provide quick-reference materials for commands and troubleshooting

**User Story:** As an industry practitioner, I need quick-reference guides for ROS 2 commands, Isaac Sim operations, and common troubleshooting scenarios, so that I can resolve issues without searching through full chapters.

**Independent Test Criteria:**
- [ ] ROS 2 command cheat sheet is complete and searchable
- [X] Troubleshooting guide covers 10+ common errors with solutions
- [ ] Notation guide explains all mathematical symbols used
- [ ] External resources are categorized and linked

**Tasks:**

- [X] T049 [P] [US5] Create docs/reference/ros2-quick-reference.md with command cheat sheet (ros2 run, ros2 topic, ros2 service, ros2 bag, colcon build) with examples
- [X] T050 [P] [US5] Create docs/reference/troubleshooting.md with 10+ common errors (ROS 2 setup, Gazebo crashes, Isaac Sim GPU errors, network issues) and solutions with related chapter links
- [X] T051 [P] [US5] Create docs/reference/notation.md with mathematical notation guide (vectors, matrices, transformations, quaternions, homogeneous coordinates)
- [X] T052 [P] [US5] Create docs/reference/external-resources.md with categorized links (official docs, research papers, community forums, hardware vendors)
- [X] T053 [US5] Create docs/reference/index.md with reference material overview and search guidance

---

### Phase 8: Polish & Cross-Cutting Concerns

**Goal:** Complete documentation, validation, deployment, and quality assurance

**Acceptance Criteria:**
- [ ] All quality gates pass (build, links, spell check, accessibility)
- [ ] Lighthouse scores ≥ 90 (performance, accessibility, SEO)
- [ ] Documentation is complete (deployment guide, authoring guide, component API)
- [ ] GitHub Actions workflow deploys successfully

**Tasks:**

- [X] T054 [P] Create PrerequisiteGraph component in src/components/PrerequisiteGraph.tsx using React Flow and dagre for auto-layout
- [X] T055 [P] Create chapter template in .specify/templates/chapter-template.md with frontmatter example (all required and optional fields)
- [X] T056 [P] Create validation plugin in plugins/validate-metadata/index.js to check frontmatter schema (week 1-13, module 1-4, prerequisites exist, estimated_time > 0)
- [X] T057 Create docs/instructors/index.md with customization guide (how to reorder chapters, adapt for semester vs. quarter)
- [X] T058 [P] Create docs/instructors/customization.md with prerequisite dependency graph and safe reordering guidelines
- [X] T059 [P] Create docs/instructors/lab-exercises.md with suggested lab activities per module
- [X] T060 Configure Algolia DocSearch in docusaurus.config.ts (apiKey placeholder, indexName, appId) with custom metadata indexing (week, module, topic)
- [X] T061 [P] Create .github/workflows/deploy.yml with build → test:links → lint → spell-check → deploy to GitHub Pages
- [X] T062 [P] Install and configure link checker: `npm install --save-dev docusaurus-plugin-broken-links-detector` and add to plugins array
- [X] T063 [P] Install and configure spell checker: `npm install --save-dev cspell` and create cspell.json with robotics dictionary
- [X] T064 [P] Create README.md in project root with project overview, development setup, build commands, contribution guidelines
- [X] T065 [P] Create docs/deployment.md with Algolia setup instructions, GitHub Pages configuration, custom domain setup (optional)
- [X] T066 [P] Create docs/components.md documenting custom component APIs (ModuleCard, QuickLinksPanel, GlossarySearch, PrerequisiteGraph)
- [X] T067 [P] Create docs/authoring-guide.md with instructions for writing chapters, using templates, frontmatter schema, code examples
- [X] T068 Run Lighthouse audit on built site and optimize images in static/img/ to <200KB each using imagemin or svgo
- [X] T069 Test mobile responsiveness on viewport widths: 375px, 768px, 1024px, 1440px and fix layout issues
- [X] T070 Test dark mode rendering for all custom components and fix any contrast/visibility issues
- [X] T071 Run accessibility audit (WCAG AA compliance) and fix any violations (alt text, color contrast, keyboard navigation)
- [X] T072 Validate that total estimated_time across all chapters sums to 130-156 hours (SC-006) using validation plugin
- [X] T073 Verify all 10 success criteria from spec.md (SC-001 through SC-010) are met and document evidence
- [X] T074 Create static/img/course-structure/prerequisite-dependency-graph.svg showing all chapter prerequisite relationships
- [X] T075 Deploy to GitHub Pages staging environment and perform end-to-end testing (navigation, search, mobile, dark mode)

---

## Task Summary

**Total Tasks:** 75

**Tasks by Phase:**
- Phase 1 (Setup): 5 tasks
- Phase 2 (Foundational): 6 tasks
- Phase 3 (US1 - P1): 9 tasks
- Phase 4 (US2 - P1): 10 tasks
- Phase 5 (US3 - P2): 8 tasks
- Phase 6 (US4 - P2): 10 tasks
- Phase 7 (US5 - P3): 5 tasks
- Phase 8 (Polish): 22 tasks

**Tasks by User Story:**
- User Story 1 (Navigate Course Structure): 9 tasks
- User Story 2 (Foundational Setup): 10 tasks
- User Story 3 (Module Learning Path): 8 tasks
- User Story 4 (Assessment Guidelines): 10 tasks
- User Story 5 (Quick References): 5 tasks
- Cross-cutting (Setup + Polish): 33 tasks

**Parallel Opportunities:**
- 42 tasks marked with [P] can be executed in parallel
- Maximum parallelization: 8 tasks simultaneously (Phase 8)
- Critical path: T001 → T006 → T010 → T017 → T035 → T042 → T073 (sequential dependencies)

**MVP Scope (User Story 1 only):**
Tasks T001-T020 (20 tasks) deliver navigable course structure with homepage dashboard

**Suggested Execution Order:**
1. Complete Phase 1-2 sequentially (foundational setup)
2. Execute Phase 3 (US1) and Phase 4 (US2) in parallel (independent features)
3. Execute Phase 5 (US3) after Phase 3 completes
4. Execute Phase 6 (US4) after Phase 5 completes
5. Execute Phase 7 (US5) anytime after Phase 2 (independent)
6. Execute Phase 8 (Polish) after all user stories complete

---

**Status:** Draft
**Version:** 1.0.0
**Last Updated:** 2025-12-09
