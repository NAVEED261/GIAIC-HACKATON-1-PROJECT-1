---
feature: Physical AI Textbook Structure
version: 1.0.0
status: Draft
created: 2025-12-09
updated: 2025-12-09
author: System
---

# Implementation Plan: Physical AI Textbook Structure

## Executive Summary

This implementation plan defines the architectural approach for creating a 13-week Physical AI course textbook using Docusaurus as the static site generator and GitHub Pages for deployment. The plan focuses on establishing the content organizational framework, navigation system, metadata schema, and custom components needed to support multiple access patterns (by week, module, topic) while maintaining prerequisite tracking and enabling incremental content delivery.

**Key Deliverables:**
1. Docusaurus project structure with configured sidebar navigation
2. Metadata schema for chapter frontmatter (week, module, prerequisites, learning objectives)
3. Homepage dashboard with 4 module cards and quick links
4. Custom glossary search component
5. Directory structure for 13 weeks of content across 4 modules
6. Template files for consistent chapter authoring
7. Prerequisite dependency graph visualization
8. Build configuration with quality gates (link checking, metadata validation)

## Scope and Dependencies

### In Scope

**Content Structure:**
- Directory hierarchy for Introduction (Weeks 1-2), Modules 1-4, Capstone Guide, Assessments, References
- Sidebar configuration supporting nested collapsible categories
- Metadata schema definition and validation
- Chapter template files

**Navigation & Discovery:**
- Single sidebar with week/module/topic access patterns
- Homepage dashboard layout with module cards
- Breadcrumbs and table of contents automation
- Cross-reference system for prerequisites

**Search & Reference:**
- Algolia DocSearch configuration plan
- Custom glossary search component specification
- Quick reference pages (ROS 2 commands, notation, troubleshooting)

**Quality & Deployment:**
- Build pipeline configuration (lint, link check, spell check)
- GitHub Actions workflow for deployment
- Performance optimization plan (image compression, lazy loading)

### Out of Scope

- Actual chapter content authoring (separate specs per chapter/module)
- Algolia account setup and API key management (deployment concern)
- Custom domain DNS configuration (deployment concern)
- Content versioning strategy (future enhancement)
- PDF export functionality (future enhancement)

### Dependencies

#### Internal Dependencies

- **Constitution Principles:** Must comply with Principle III (Consistency & Standards), IV (Docusaurus Structure & Quality), VII (Deployment & Publishing Standards)
- **Existing Templates:** Spec template, plan template, tasks template from `.specify/templates/`
- **Design System:** `docs/design-system.md` defines colors, typography, spacing for custom components

#### External Dependencies

- **Docusaurus v3:** Static site generator framework ([docusaurus.io](https://docusaurus.io))
- **React 18.x:** Required by Docusaurus for custom components
- **Algolia DocSearch:** Search indexing service (requires account and crawler configuration)
- **GitHub Pages:** Hosting platform (automatic deployment from `main` branch)
- **Node.js 18+:** Runtime for Docusaurus build process

## Architecture & Design Decisions

### Decision 1: Docusaurus as Static Site Generator

**Options Considered:**
1. Docusaurus v3 (React-based, optimized for documentation)
2. VuePress (Vue-based, simpler but less extensible)
3. GitBook (Hosted service, limited customization)
4. Custom Next.js site (Maximum flexibility, higher maintenance)

**Trade-offs:**
- Docusaurus: Best balance of features (sidebar, MDX, search integration) vs. complexity
- VuePress: Simpler but lacks advanced features like custom React components
- GitBook: Easiest setup but vendor lock-in and limited UI customization
- Next.js: Full control but requires building all documentation features from scratch

**Rationale:**
Docusaurus v3 is chosen because:
- Built specifically for technical documentation with navigation, versioning, and search
- Supports MDX for embedding React components (needed for glossary search, module cards)
- Active ecosystem with plugins for link checking, image optimization, Algolia integration
- Aligns with Constitution Principle IV (Docusaurus Structure & Quality)
- GitHub Pages deployment is well-documented and supported

**ADR:** Create ADR-001 for this decision (impacts all future development)

### Decision 2: Single Sidebar with Nested Categories vs. Multiple Sidebars

**Options Considered:**
1. Single sidebar with nested collapsible categories (Introduction → Module 1 → Module 2 → ...)
2. Multiple sidebars (one per module) with tab switching
3. Sidebar + top navigation bar combo (modules in top nav, chapters in sidebar)

**Trade-offs:**
- Single sidebar: Simpler navigation model, easier to see overall structure, can become long
- Multiple sidebars: Cleaner per-module view but harder to see cross-module relationships
- Hybrid: Best of both but adds UI complexity and cognitive load

**Rationale:**
Single sidebar with nested categories chosen because:
- Satisfies requirement FR-008 (access by week, module, topic)
- Students can see prerequisite chains across modules at a glance
- Docusaurus auto-collapse feature keeps sidebar manageable
- Aligns with Clarification Q1 (single sidebar with nested collapsible categories)

**ADR:** Document in ADR-002 if significant complexity emerges

### Decision 3: Metadata Schema for Chapter Frontmatter

**Options Considered:**
1. Docusaurus defaults only (title, description, sidebar_position)
2. Docusaurus defaults + custom fields (week, module, prerequisites, learning_objectives, estimated_time)
3. Fully custom schema with validation library

**Trade-offs:**
- Defaults only: Simple but lacks prerequisite tracking and time estimation
- Defaults + custom: Balanced approach, leverages Docusaurus while adding needed fields
- Fully custom: Maximum control but requires building custom rendering logic

**Rationale:**
Docusaurus defaults + custom fields chosen because:
- Satisfies FR-011 (estimated_time, week, module, prerequisites, learning_objectives)
- Enables automated prerequisite validation and time summation
- Can be validated at build time using custom Docusaurus plugin
- Supports incremental addition of optional fields (assessment_type, difficulty_level, capstone_component)

**Schema Design:**
```yaml
---
title: "Chapter Title"
description: "Chapter description for SEO"
sidebar_label: "Short Label"
sidebar_position: 1
estimated_time: 3  # hours
week: 3  # 1-13
module: 1  # 1-4
prerequisites: ["week-2-intro", "week-2-setup"]
learning_objectives:
  - "Objective 1"
  - "Objective 2"
# Optional fields
assessment_type: "ros2-package"  # if chapter contributes to assessment
difficulty_level: "intermediate"  # beginner | intermediate | advanced
capstone_component: "navigation"  # voice | plan | navigate | perceive | manipulate
---
```

**ADR:** None needed (follows standard Docusaurus pattern)

### Decision 4: Custom Homepage vs. Docusaurus Default Landing

**Options Considered:**
1. Custom React component for homepage dashboard (module cards, quick links, updates)
2. Standard Docusaurus landing page with markdown content
3. Hybrid: Markdown content with embedded React components

**Trade-offs:**
- Custom component: Full design control but requires React/TypeScript development
- Standard landing: Fastest to create but limited layout options
- Hybrid: Good balance but can feel disjointed if not carefully designed

**Rationale:**
Custom React component chosen because:
- Satisfies FR-013 (dashboard-style layout with module cards, quick links, recent updates)
- Provides engaging entry point for students (visual cards vs. text-heavy page)
- Allows dynamic data (e.g., estimated time pulled from chapter metadata)
- Aligns with Clarification Q3 (dashboard-style with module cards + quick links sidebar)

**Implementation:**
- Create `src/pages/index.tsx` with custom layout
- Use `ModuleCard` component for 4 module cards
- Use `QuickLinks` component for sidebar
- Use `RecentUpdates` component for changelog section

**ADR:** None needed (standard Docusaurus customization pattern)

### Decision 5: Glossary Search Implementation

**Options Considered:**
1. Dedicated React component with local JSON data (instant search, no external dependency)
2. Algolia DocSearch covering both content and glossary (unified search experience)
3. Hybrid: Algolia for content, custom component for glossary (best of both)

**Trade-offs:**
- Dedicated component: Fast, no API dependency, but requires custom UI development
- Algolia only: Consistent search UX but glossary terms may not rank well in general search
- Hybrid: Best user experience but more complex to implement and maintain

**Rationale:**
Hybrid approach chosen because:
- Satisfies FR-007a (Algolia DocSearch for main content + dedicated glossary search component)
- Aligns with Clarification Q4 (hybrid approach with custom glossary component)
- Glossary search needs instant response (<2s per SC-007), which is better with local data
- General content search benefits from Algolia's relevance ranking and full-text capabilities

**Implementation:**
- Create `GlossarySearch` React component with Fuse.js for fuzzy matching
- Store glossary terms in `static/data/glossary.json`
- Embed glossary search in sidebar or as floating widget
- Configure Algolia to index main content (chapters, setup guides, assessments)

**ADR:** Create ADR-003 if implementation complexity requires significant engineering trade-offs

## Content Structure

### Directory Organization

```
docs/
├── intro/                          # Weeks 1-2: Introduction
│   ├── index.md                   # Physical AI Overview
│   ├── what-is-physical-ai.md
│   ├── course-structure.md
│   └── learning-outcomes.md
├── setup/                          # Foundational Setup
│   ├── index.md                   # Setup Overview
│   ├── hardware-digital-twin.md   # Option 1: Workstation
│   ├── hardware-edge-kit.md       # Option 2: Jetson Orin Nano
│   ├── hardware-cloud.md          # Option 3: Cloud (AWS/Azure)
│   ├── software-ros2.md
│   └── software-isaac-sim.md
├── module-1-ros2/                  # Weeks 3-5: ROS 2
│   ├── index.md                   # Module Overview
│   ├── week-3-basics/
│   │   ├── ros2-fundamentals.md
│   │   └── nodes-topics.md
│   ├── week-4-communication/
│   │   ├── services.md
│   │   └── actions.md
│   └── week-5-integration/
│       └── multi-node-systems.md
├── module-2-digital-twin/          # Weeks 6-7: Digital Twin
│   ├── index.md
│   ├── week-6-gazebo/
│   │   ├── simulation-basics.md
│   │   └── robot-models.md
│   └── week-7-unity/
│       └── ml-agents.md
├── module-3-isaac/                 # Weeks 8-10: NVIDIA Isaac
│   ├── index.md
│   ├── week-8-setup/
│   ├── week-9-perception/
│   └── week-10-manipulation/
├── module-4-vla/                   # Weeks 11-13: VLA & Humanoids
│   ├── index.md
│   ├── week-11-vla-intro/
│   ├── week-12-humanoid-integration/
│   └── week-13-capstone-prep/
├── capstone/                       # Capstone Project Guide
│   ├── index.md                   # Architecture Overview
│   ├── voice-interface.md
│   ├── task-planning.md
│   ├── navigation.md
│   ├── perception.md
│   └── manipulation.md
├── assessments/                    # Assessment Guides
│   ├── index.md
│   ├── ros2-package-project.md
│   ├── gazebo-simulation-project.md
│   ├── isaac-perception-project.md
│   └── capstone-project.md
├── reference/                      # Reference Materials
│   ├── glossary.md                # 100+ terms
│   ├── notation.md                # Mathematical symbols
│   ├── ros2-quick-reference.md    # Command cheat sheet
│   └── troubleshooting.md         # Common errors
└── instructors/                    # Instructor Guide
    ├── index.md
    ├── customization.md
    └── lab-exercises.md
```

### Sidebar Configuration (sidebars.ts)

```typescript
module.exports = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction (Weeks 1-2)',
      collapsed: false,
      items: ['intro/index', 'intro/what-is-physical-ai', 'intro/course-structure'],
    },
    {
      type: 'category',
      label: 'Foundational Setup',
      collapsed: false,
      items: [
        'setup/index',
        {
          type: 'category',
          label: 'Hardware Options',
          items: ['setup/hardware-digital-twin', 'setup/hardware-edge-kit', 'setup/hardware-cloud'],
        },
        'setup/software-ros2',
        'setup/software-isaac-sim',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 (Weeks 3-5)',
      collapsed: true,
      items: [
        'module-1-ros2/index',
        {
          type: 'category',
          label: 'Week 3: Basics',
          items: ['module-1-ros2/week-3-basics/ros2-fundamentals', 'module-1-ros2/week-3-basics/nodes-topics'],
        },
        {
          type: 'category',
          label: 'Week 4: Communication',
          items: ['module-1-ros2/week-4-communication/services', 'module-1-ros2/week-4-communication/actions'],
        },
        {
          type: 'category',
          label: 'Week 5: Integration',
          items: ['module-1-ros2/week-5-integration/multi-node-systems'],
        },
      ],
    },
    // ... Module 2, 3, 4 similarly structured
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/index',
        'capstone/voice-interface',
        'capstone/task-planning',
        'capstone/navigation',
        'capstone/perception',
        'capstone/manipulation',
      ],
    },
    {
      type: 'category',
      label: 'Assessments',
      items: [
        'assessments/index',
        'assessments/ros2-package-project',
        'assessments/gazebo-simulation-project',
        'assessments/isaac-perception-project',
        'assessments/capstone-project',
      ],
    },
    {
      type: 'category',
      label: 'Reference',
      items: ['reference/glossary', 'reference/notation', 'reference/ros2-quick-reference', 'reference/troubleshooting'],
    },
    {
      type: 'category',
      label: 'For Instructors',
      items: ['instructors/index', 'instructors/customization', 'instructors/lab-exercises'],
    },
  ],
};
```

## Technical Implementation

### Custom React Components

#### Component 1: ModuleCard

**File:** `src/components/ModuleCard.tsx`

**Purpose:** Display module information on homepage dashboard (title, week range, learning outcomes, estimated time)

**Props:**
```typescript
interface ModuleCardProps {
  moduleNumber: number;
  title: string;
  weekRange: string;
  learningOutcomes: string[];
  estimatedHours: number;
  link: string;
}
```

**Implementation Notes:**
- Use card design from design-system.md
- Display module number badge (1, 2, 3, 4)
- Show week range prominently ("Weeks 3-5")
- List 3-5 key learning outcomes
- Display estimated time ("~30 hours")
- Clickable to navigate to module index page

#### Component 2: QuickLinksPanel

**File:** `src/components/QuickLinksPanel.tsx`

**Purpose:** Sidebar panel with quick links to hardware setup, assessments, glossary

**Props:**
```typescript
interface QuickLinksProps {
  links: Array<{
    label: string;
    url: string;
    icon?: string;
  }>;
}
```

**Implementation:**
- Fixed position sidebar (right side on desktop, drawer on mobile)
- Icon + label for each link
- Default links: Hardware Setup, Assessments, Glossary, Troubleshooting

#### Component 3: GlossarySearch

**File:** `src/components/GlossarySearch.tsx`

**Purpose:** Instant glossary term lookup with autocomplete

**Props:**
```typescript
interface GlossarySearchProps {
  glossaryData: GlossaryEntry[];
}

interface GlossaryEntry {
  term: string;
  definition: string;
  relatedChapters: string[];
}
```

**Implementation:**
- Use Fuse.js for fuzzy string matching
- Autocomplete dropdown with term suggestions
- Display definition and related chapter links
- Debounce search input (300ms delay)
- Load glossary data from `static/data/glossary.json`

#### Component 4: PrerequisiteGraph

**File:** `src/components/PrerequisiteGraph.tsx`

**Purpose:** Visual dependency graph showing prerequisite relationships

**Props:**
```typescript
interface PrerequisiteGraphProps {
  chapters: Array<{
    id: string;
    title: string;
    week: number;
    prerequisites: string[];
  }>;
}
```

**Implementation:**
- Use D3.js or React Flow for graph visualization
- Nodes represent chapters, edges represent prerequisites
- Color-code by module
- Highlight critical path to capstone
- Interactive (click node to navigate to chapter)

### Visual Assets

#### Diagram 1: Course Structure Overview

**File:** `static/img/course-structure-overview.svg`
**Type:** Timeline diagram
**Tool:** draw.io or Figma
**Purpose:** Visual representation of 13-week progression through 4 modules
**Alt Text:** "13-week Physical AI course timeline showing Introduction (Weeks 1-2), Module 1 ROS 2 (Weeks 3-5), Module 2 Digital Twin (Weeks 6-7), Module 3 Isaac (Weeks 8-10), Module 4 VLA (Weeks 11-13), with capstone project integration points"

#### Diagram 2: Prerequisite Dependency Graph

**File:** `static/img/prerequisite-dependency-graph.svg`
**Type:** Directed acyclic graph (DAG)
**Tool:** Graphviz or draw.io
**Purpose:** Show how chapters build on each other (prerequisite chains)
**Alt Text:** "Prerequisite dependency graph showing learning pathways from Introduction through Module 1-4 to Capstone, with arrows indicating prerequisite relationships"

#### Diagram 3: Capstone Architecture

**File:** `static/img/capstone-architecture.svg`
**Type:** System architecture diagram
**Purpose:** 5-component architecture (voice → plan → navigate → perceive → manipulate)
**Alt Text:** "Autonomous humanoid capstone project architecture with 5 components: voice interface for input, task planner for decision making, navigation for movement, perception for environment sensing, and manipulation for object interaction"

## UI/UX Considerations

### Homepage Dashboard Layout

```
+--------------------------------------------------+
|  Header (Logo | Search | GitHub)                 |
+--------------------------------------------------+
|                                                  |
|  Hero Section                                    |
|  - Title: "Physical AI Textbook"                 |
|  - Subtitle: "13-Week Course in Humanoid..."     |
|  - CTA: "Start Learning" button                  |
|                                                  |
+--------------------------------------------------+
|  Module Cards (Grid: 2x2)                        |
|  +------------+  +------------+                  |
|  | Module 1   |  | Module 2   |                  |
|  | ROS 2      |  | Digital    |                  |
|  | Weeks 3-5  |  | Twin 6-7   |                  |
|  | 30 hours   |  | 20 hours   |                  |
|  +------------+  +------------+                  |
|  +------------+  +------------+                  |
|  | Module 3   |  | Module 4   |                  |
|  | Isaac 8-10 |  | VLA 11-13  |                  |
|  | 30 hours   |  | 30 hours   |                  |
|  +------------+  +------------+                  |
+--------------------------------------------------+
|  Quick Links (Sidebar or Below)                  |
|  - Hardware Setup                                |
|  - Assessments                                   |
|  - Glossary                                      |
|  - Troubleshooting                               |
+--------------------------------------------------+
|  Recent Updates (Optional)                       |
|  - Change log entries                            |
+--------------------------------------------------+
```

### Theme Customization

**Custom CSS:** `src/css/custom.css`

```css
/* Module card styling */
.module-card {
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 12px;
  padding: 24px;
  transition: box-shadow 0.2s;
}

.module-card:hover {
  box-shadow: 0 4px 16px rgba(0, 0, 0, 0.1);
}

/* Quick links panel */
.quick-links-panel {
  position: fixed;
  right: 24px;
  top: 100px;
  width: 200px;
  background: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  padding: 16px;
}

/* Prerequisite badge */
.prerequisite-badge {
  display: inline-block;
  background: var(--ifm-color-primary-lightest);
  color: var(--ifm-color-primary-darkest);
  padding: 4px 8px;
  border-radius: 4px;
  font-size: 12px;
  margin: 2px;
}
```

**Colors:** Use design system colors from `docs/design-system.md`
- Primary: `#3578e5` (Docusaurus Blue)
- Secondary: `#25c2a0` (Teal)
- Module 1: `#3578e5` (Blue)
- Module 2: `#8e44ad` (Purple)
- Module 3: `#e67e22` (Orange)
- Module 4: `#27ae60` (Green)

**Typography:**
- Headings: `'Inter', sans-serif`
- Body: `'Inter', sans-serif`
- Code: `'Fira Code', monospace`

## Quality & Testing

### Testing Strategy

- [ ] **Docusaurus build:** `npm run build` completes without errors or warnings
- [ ] **Link validation:** All internal links resolve (use `docusaurus-plugin-broken-links-detector`)
- [ ] **Metadata validation:** Custom plugin validates chapter frontmatter schema
- [ ] **Image optimization:** All images in `static/img/` < 200KB (use `imagemin` or `svgo`)
- [ ] **Mobile responsive:** Test on viewport widths 375px, 768px, 1024px, 1440px
- [ ] **Dark mode:** All custom components render correctly in dark theme
- [ ] **Accessibility:** Lighthouse accessibility score ≥ 90, WCAG AA compliance
- [ ] **Spell check:** No typos in published content (use `cspell` or similar)
- [ ] **Performance:** Lighthouse performance score ≥ 90, LCP < 2.5s, CLS < 0.1

### Review Checkpoints

1. **Technical Review** (Curriculum Designer)
   - Course structure aligns with learning outcomes
   - Prerequisite chains are logical and complete
   - Module progression supports capstone project

2. **UX Review** (Designer)
   - Homepage dashboard is engaging and clear
   - Navigation is intuitive (2-click rule satisfied)
   - Mobile experience is usable

3. **Editorial Review** (Content Editor)
   - Metadata schema is consistent
   - Glossary structure is well-organized
   - Reference materials are comprehensive

### Build Pipeline (GitHub Actions)

```yaml
# .github/workflows/deploy.yml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - run: npm ci
      - run: npm run build
      - run: npm run test:links  # Link checker
      - run: npm run lint  # ESLint + Prettier
      - run: npm run spell-check  # Spell checker
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

## Risk Analysis

### Top Risks

1. **Sidebar Becomes Too Long with 13 Weeks of Content**
   - **Impact:** Medium (navigation becomes overwhelming)
   - **Probability:** High (13 weeks × average 3-4 chapters/week = 40-50 items)
   - **Mitigation:**
     - Use Docusaurus auto-collapse feature (only expand current module)
     - Add "Jump to Week" dropdown selector
     - Provide alternative "By Topic" view (kinematics, perception, control)

2. **Prerequisite Validation Complexity**
   - **Impact:** Medium (broken prerequisite links cause confusion)
   - **Probability:** Medium (manual authoring errors)
   - **Mitigation:**
     - Build custom Docusaurus plugin to validate prerequisites at build time
     - Create script to auto-generate prerequisite graph from frontmatter
     - Include prerequisite validation in CI/CD pipeline (fail build on missing prerequisites)

3. **Algolia DocSearch Crawler Configuration**
   - **Impact:** High (search doesn't work if misconfigured)
   - **Probability:** Medium (requires external service setup)
   - **Mitigation:**
     - Document Algolia setup process in `docs/deployment.md`
     - Provide fallback search using local index (Lunr.js) for development
     - Test search with realistic query scenarios before launch

4. **Custom Components Maintenance Burden**
   - **Impact:** Medium (components break on Docusaurus updates)
   - **Probability:** Low (Docusaurus v3 is stable)
   - **Mitigation:**
     - Pin Docusaurus and React versions in package.json
     - Write unit tests for custom components (Jest + React Testing Library)
     - Document component API in `docs/components.md`

5. **Metadata Schema Evolution**
   - **Impact:** Low (adding fields breaks existing chapters)
   - **Probability:** Medium (requirements change during development)
   - **Mitigation:**
     - Make new fields optional with sensible defaults
     - Version metadata schema (v1, v2) if breaking changes needed
     - Provide migration script for bulk frontmatter updates

## Complexity Justifications

### Deviation 1: Custom React Components for Homepage

**Principle Violated:** Simplicity (custom components add complexity vs. markdown-only content)

**Why Necessary:** FR-013 requires dashboard-style layout with module cards, quick links, and recent updates. Standard Docusaurus markdown pages cannot achieve this layout without custom components.

**Alternative Rejected:** Using only markdown with HTML/CSS inline was considered but would be harder to maintain and wouldn't support dynamic data (e.g., pulling estimated time from chapter metadata).

**Migration Path:** If components become unmaintainable, consider migrating to Docusaurus v4 (when available) or alternative frameworks with better built-in dashboard layouts.

### Deviation 2: Hybrid Search (Algolia + Custom Glossary)

**Principle Violated:** Simplicity (two search implementations instead of one)

**Why Necessary:** FR-007a explicitly requires hybrid approach. Glossary search needs instant response (<2s per SC-007), which is better achieved with local data. General content search benefits from Algolia's relevance ranking.

**Alternative Rejected:** Using only Algolia for both content and glossary was considered but glossary terms may not rank well in general search results (drowned out by chapter content).

**Migration Path:** If Algolia proves sufficient for both use cases, remove custom glossary component and rely solely on Algolia with custom ranking configuration.

## Timeline & Milestones

**Note:** Milestones define completion criteria, not time estimates.

### Phase 0: Setup & Research (Complete)
- [x] Specification approved
- [x] Planning initiated
- [ ] Research.md created (Docusaurus best practices, custom components patterns)

### Phase 1: Project Initialization
- [ ] Docusaurus project scaffolded (`npx create-docusaurus@latest`)
- [ ] Directory structure created (docs/, static/, src/components/)
- [ ] Sidebar configuration drafted (sidebars.ts)
- [ ] Metadata schema defined and documented

### Phase 2: Core Components
- [ ] Homepage dashboard implemented (ModuleCard, QuickLinksPanel)
- [ ] Glossary search component implemented (GlossarySearch)
- [ ] Prerequisite graph visualization implemented (PrerequisiteGraph)
- [ ] Custom CSS theme applied

### Phase 3: Content Templates
- [ ] Chapter template created with frontmatter examples
- [ ] Module index template created
- [ ] Assessment template created
- [ ] Reference page templates created (glossary, notation, troubleshooting)

### Phase 4: Build & Validation
- [ ] Link checker integrated (`docusaurus-plugin-broken-links-detector`)
- [ ] Metadata validation plugin implemented
- [ ] Spell checker integrated (`cspell`)
- [ ] GitHub Actions workflow configured

### Phase 5: Documentation
- [ ] `docs/deployment.md` created (Algolia setup, GitHub Pages configuration)
- [ ] `docs/components.md` created (custom component API documentation)
- [ ] `docs/authoring-guide.md` created (how to write chapters, use templates)
- [ ] `README.md` updated with project overview

### Phase 6: Testing & Deployment
- [ ] Build validation passes (no errors/warnings)
- [ ] Lighthouse audit scores ≥ 90 (performance, accessibility, SEO)
- [ ] Mobile responsive tested
- [ ] Dark mode tested
- [ ] Deployed to GitHub Pages staging environment

## Success Criteria

This implementation is complete when:

- [ ] All 10 success criteria from specification (SC-001 through SC-010) are met
- [ ] Docusaurus project builds successfully without warnings
- [ ] Homepage dashboard displays 4 module cards with accurate data
- [ ] Sidebar navigation supports access by week, module, and topic
- [ ] Glossary search component provides instant lookup (<2 seconds)
- [ ] Prerequisite dependency graph is generated from chapter metadata
- [ ] All quality gates pass (link check, spell check, accessibility audit)
- [ ] GitHub Pages deployment pipeline is functional
- [ ] Documentation is complete (deployment, components, authoring guide)
- [ ] First chapter (Introduction) can be authored using templates and structure

## Related Documentation

- **Spec:** `specs/1-physical-ai-textbook-structure/spec.md`
- **Research:** `specs/1-physical-ai-textbook-structure/research/research.md` (to be created)
- **Data Model:** `specs/1-physical-ai-textbook-structure/data-model.md` (to be created)
- **Contracts:** `specs/1-physical-ai-textbook-structure/contracts/` (metadata schema, component interfaces)
- **ADRs:** To be created for significant architectural decisions

---

**Status:** Draft
**Version:** 1.0.0
**Last Updated:** 2025-12-09
