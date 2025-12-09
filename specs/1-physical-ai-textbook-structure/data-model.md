# Data Model: Physical AI Textbook Structure

**Date:** 2025-12-09
**Feature:** Physical AI Textbook Structure
**Version:** 1.0.0

## Purpose

This document defines the data structures and relationships for the Physical AI textbook content organization system. It specifies frontmatter metadata schemas, component interfaces, and data contracts used throughout the Docusaurus site.

## Entity Relationships

```
Part
 ├── Module (1:many)
 │   └── Chapter (1:many)
 │       ├── Prerequisites (many:many with other Chapters)
 │       ├── Learning Objectives (1:many)
 │       └── Assessment (many:1, optional)
 ├── Assessment (1:many)
 │   ├── Rubric (1:1)
 │   └── Related Chapters (many:many)
 └── Reference Material (1:many)
     └── Glossary Entry (1:many)
         └── Related Chapters (many:many)
```

## Core Entities

### 1. Part

Represents a high-level content grouping (e.g., Introduction, Module 1, Capstone).

**Type Definition:**
```typescript
interface Part {
  id: string;  // e.g., "introduction", "module-1-ros2"
  label: string;  // e.g., "Introduction (Weeks 1-2)"
  type: 'intro' | 'setup' | 'module' | 'capstone' | 'assessment' | 'reference' | 'instructor';
  weekRange?: string;  // e.g., "1-2" (for intro), "3-5" (for modules)
  modules?: Module[];  // If type is 'module'
  order: number;  // Sidebar position
}
```

**Example:**
```typescript
{
  id: "module-1-ros2",
  label: "Module 1: ROS 2 (Weeks 3-5)",
  type: "module",
  weekRange: "3-5",
  modules: [/* Module objects */],
  order: 3
}
```

### 2. Module

Represents a major course section (ROS 2, Digital Twin, Isaac, VLA).

**Type Definition:**
```typescript
interface Module {
  id: string;  // e.g., "module-1-ros2"
  title: string;  // e.g., "ROS 2 Fundamentals"
  moduleNumber: number;  // 1-4
  weekRange: string;  // e.g., "3-5"
  estimatedHours: number;  // Total time for module
  learningOutcomes: string[];  // 3-5 key outcomes
  chapters: Chapter[];  // All chapters in this module
  capstoneIntegration?: string;  // How module contributes to capstone
  color?: string;  // Module color for UI (hex code)
}
```

**Example:**
```typescript
{
  id: "module-1-ros2",
  title: "ROS 2 Fundamentals",
  moduleNumber: 1,
  weekRange: "3-5",
  estimatedHours: 30,
  learningOutcomes: [
    "Understand ROS 2 architecture and concepts",
    "Create ROS 2 nodes, topics, services, and actions",
    "Build multi-node robotic systems"
  ],
  chapters: [/* Chapter objects */],
  capstoneIntegration: "Provides communication layer for all capstone components",
  color: "#3578e5"
}
```

### 3. Chapter

Represents a single topic within a module (maps to one markdown file).

**Type Definition (Frontmatter Schema):**
```typescript
interface ChapterFrontmatter {
  // Standard Docusaurus fields
  title: string;  // Chapter title
  description: string;  // SEO description
  sidebar_label: string;  // Short label for sidebar
  sidebar_position: number;  // Order within parent category
  slug?: string;  // Custom URL (optional)
  tags?: string[];  // For categorization

  // Custom fields (required)
  estimated_time: number;  // Hours to complete
  week: number;  // 1-13
  module: number;  // 1-4
  prerequisites: string[];  // Array of chapter IDs
  learning_objectives: string[];  // 3-5 measurable objectives

  // Custom fields (optional)
  assessment_type?: 'ros2-package' | 'gazebo-simulation' | 'isaac-perception' | 'capstone';
  difficulty_level?: 'beginner' | 'intermediate' | 'advanced';
  capstone_component?: 'voice' | 'plan' | 'navigate' | 'perceive' | 'manipulate';
  draft?: boolean;  // If true, excluded from production build

  // Metadata for internal use
  last_updated?: string;  // ISO date
  author?: string;  // Author name
}
```

**Example:**
```yaml
---
title: "ROS 2 Nodes and Topics"
description: "Learn how to create ROS 2 nodes and communicate using topics"
sidebar_label: "Nodes & Topics"
sidebar_position: 2
estimated_time: 3
week: 3
module: 1
prerequisites:
  - "module-1-ros2/week-3-basics/ros2-fundamentals"
learning_objectives:
  - "Create a ROS 2 node using rclpy"
  - "Publish and subscribe to topics"
  - "Understand Quality of Service (QoS) settings"
difficulty_level: "beginner"
---
```

**Validation Rules:**
- `estimated_time`: Must be > 0 and < 20 (hours)
- `week`: Must be 1-13
- `module`: Must be 1-4
- `prerequisites`: Each ID must reference a valid chapter
- `learning_objectives`: Must have at least 1 objective
- `draft`: If true, chapter not included in production build

### 4. Assessment

Represents a project or evaluation point.

**Type Definition:**
```typescript
interface Assessment {
  id: string;  // e.g., "ros2-package-project"
  title: string;  // e.g., "ROS 2 Package Development"
  type: 'ros2-package' | 'gazebo-simulation' | 'isaac-perception' | 'capstone';
  relatedChapters: string[];  // Chapter IDs that prepare for this assessment
  rubric: Rubric;
  requirements: string[];  // List of deliverables
  submissionGuidelines: string;  // How to submit
  estimatedHours: number;  // Time to complete
}
```

**Example:**
```typescript
{
  id: "ros2-package-project",
  title: "ROS 2 Package Development",
  type: "ros2-package",
  relatedChapters: [
    "module-1-ros2/week-3-basics/ros2-fundamentals",
    "module-1-ros2/week-3-basics/nodes-topics",
    "module-1-ros2/week-4-communication/services"
  ],
  rubric: {/* Rubric object */},
  requirements: [
    "Create a ROS 2 package with at least 2 nodes",
    "Implement topic communication between nodes",
    "Include unit tests for each node"
  ],
  submissionGuidelines: "Submit GitHub repository link with README",
  estimatedHours: 8
}
```

### 5. Rubric

Structured evaluation criteria for assessments.

**Type Definition:**
```typescript
interface Rubric {
  criteria: RubricCriterion[];
}

interface RubricCriterion {
  name: string;  // e.g., "Code Quality"
  weight: number;  // 0-1 (sum of all weights should be 1.0)
  levels: RubricLevel[];
}

interface RubricLevel {
  level: 'needs-improvement' | 'proficient' | 'excellent';
  score: number;  // Points (e.g., 0, 1, 2)
  description: string;  // What defines this level
}
```

**Example:**
```typescript
{
  criteria: [
    {
      name: "Code Quality",
      weight: 0.3,
      levels: [
        {
          level: "needs-improvement",
          score: 0,
          description: "Code has errors or doesn't follow ROS 2 conventions"
        },
        {
          level: "proficient",
          score: 1,
          description: "Code works and follows basic ROS 2 conventions"
        },
        {
          level: "excellent",
          score: 2,
          description: "Code is clean, well-documented, and follows best practices"
        }
      ]
    },
    {
      name: "Functionality",
      weight: 0.5,
      levels: [/* ... */]
    },
    {
      name: "Testing",
      weight: 0.2,
      levels: [/* ... */]
    }
  ]
}
```

### 6. Glossary Entry

A term definition with chapter cross-references.

**Type Definition:**
```typescript
interface GlossaryEntry {
  term: string;  // Primary term
  aliases?: string[];  // Alternative names
  definition: string;  // Concise explanation
  relatedChapters: string[];  // Chapter IDs where term is used
  externalLinks?: ExternalLink[];  // Additional resources
}

interface ExternalLink {
  label: string;
  url: string;
}
```

**Example:**
```typescript
{
  term: "ROS 2",
  aliases: ["Robot Operating System 2", "ROS2"],
  definition: "Robot Operating System 2, a set of software libraries and tools for building robot applications. Successor to ROS 1 with improved real-time performance and multi-robot support.",
  relatedChapters: [
    "module-1-ros2/week-3-basics/ros2-fundamentals",
    "setup/software-ros2"
  ],
  externalLinks: [
    {
      label: "Official ROS 2 Documentation",
      url: "https://docs.ros.org/en/rolling/"
    }
  ]
}
```

### 7. Hardware Configuration

Represents one of three setup paths.

**Type Definition:**
```typescript
interface HardwareConfiguration {
  id: string;  // e.g., "digital-twin-workstation"
  name: string;  // e.g., "Digital Twin Workstation"
  type: 'workstation' | 'edge-kit' | 'cloud';
  requirements: HardwareRequirement[];
  softwareSteps: InstallationStep[];
  costEstimate: {
    min: number;  // USD
    max: number;  // USD
    currency: string;
  };
  limitations: string[];  // What doesn't work with this config
  recommended: boolean;  // Primary recommendation
}

interface HardwareRequirement {
  component: string;  // e.g., "GPU"
  specification: string;  // e.g., "NVIDIA RTX 3060 or better"
  required: boolean;
}

interface InstallationStep {
  order: number;
  title: string;
  commands?: string[];  // Shell commands
  description: string;
  verificationCommand?: string;  // How to check if step succeeded
}
```

**Example:**
```typescript
{
  id: "digital-twin-workstation",
  name: "Digital Twin Workstation",
  type: "workstation",
  requirements: [
    {
      component: "GPU",
      specification: "NVIDIA RTX 3060 or better (12GB VRAM minimum)",
      required: true
    },
    {
      component: "OS",
      specification: "Ubuntu 22.04 LTS",
      required: true
    },
    {
      component: "RAM",
      specification: "32GB or more",
      required: true
    }
  ],
  softwareSteps: [
    {
      order: 1,
      title: "Install ROS 2 Humble",
      commands: [
        "sudo apt update",
        "sudo apt install ros-humble-desktop"
      ],
      description: "Install ROS 2 Humble desktop variant",
      verificationCommand: "ros2 --version"
    }
  ],
  costEstimate: {
    min: 1500,
    max: 3000,
    currency: "USD"
  },
  limitations: [
    "Requires physical hardware (not portable)",
    "Higher upfront cost than cloud option"
  ],
  recommended: true
}
```

## Component Props

### ModuleCard Component

```typescript
interface ModuleCardProps {
  moduleNumber: number;  // 1-4
  title: string;  // e.g., "ROS 2 Fundamentals"
  weekRange: string;  // e.g., "Weeks 3-5"
  learningOutcomes: string[];  // 3-5 outcomes
  estimatedHours: number;  // Total module time
  link: string;  // Navigation URL (e.g., "/docs/module-1-ros2")
  color?: string;  // Optional custom color (hex)
}
```

### GlossarySearch Component

```typescript
interface GlossarySearchProps {
  glossaryData: GlossaryEntry[];  // Array of all terms
  placeholder?: string;  // Search input placeholder
  maxResults?: number;  // Max autocomplete results (default: 5)
}
```

### PrerequisiteGraph Component

```typescript
interface PrerequisiteGraphProps {
  chapters: ChapterNode[];  // All chapters with metadata
  highlightPath?: string[];  // Optional: highlight specific path
  onNodeClick?: (chapterId: string) => void;  // Optional: callback on node click
}

interface ChapterNode {
  id: string;
  title: string;
  week: number;
  module: number;
  prerequisites: string[];  // IDs of prerequisite chapters
}
```

### QuickLinksPanel Component

```typescript
interface QuickLinksPanelProps {
  links: QuickLink[];
  position?: 'right' | 'left';  // Default: 'right'
}

interface QuickLink {
  label: string;
  url: string;
  icon?: React.ReactNode;  // Optional icon component
  badge?: string;  // Optional badge text (e.g., "New")
}
```

## File Structure Contracts

### Glossary Data File

**Location:** `static/data/glossary.json`

**Schema:**
```json
[
  {
    "term": "string (required)",
    "aliases": ["string[]"] (optional),
    "definition": "string (required)",
    "relatedChapters": ["string[]"] (required, can be empty),
    "externalLinks": [
      {
        "label": "string",
        "url": "string"
      }
    ] (optional)
  }
]
```

**Validation:**
- Must be valid JSON array
- Each entry must have `term` and `definition`
- `relatedChapters` must reference valid chapter IDs
- Minimum 100 entries for SC-007 compliance

### Module Metadata File

**Location:** `static/data/modules.json`

**Schema:**
```json
[
  {
    "id": "string (required)",
    "title": "string (required)",
    "moduleNumber": "number (1-4, required)",
    "weekRange": "string (required)",
    "estimatedHours": "number (required)",
    "learningOutcomes": ["string[]"] (required),
    "capstoneIntegration": "string" (optional),
    "color": "string (hex code, optional)"
  }
]
```

### Sidebar Configuration

**Location:** `sidebars.ts`

**Type:**
```typescript
type Sidebar = SidebarItem[];

type SidebarItem =
  | string  // Document ID
  | {
      type: 'category';
      label: string;
      collapsed?: boolean;
      collapsible?: boolean;
      items: SidebarItem[];
    }
  | {
      type: 'link';
      label: string;
      href: string;
    };
```

## Data Validation

### Build-Time Validation

Custom Docusaurus plugin validates:

1. **Chapter Frontmatter:**
   - All required fields present
   - `week` is 1-13
   - `module` is 1-4
   - `estimated_time` is positive
   - `prerequisites` reference valid chapter IDs

2. **Prerequisite Graph:**
   - No circular dependencies
   - All prerequisites exist
   - Prerequisite weeks/modules come before dependent chapters

3. **Glossary:**
   - Minimum 100 entries
   - `relatedChapters` reference valid chapter IDs
   - No duplicate terms

4. **Module Metadata:**
   - 4 modules defined
   - Week ranges don't overlap
   - Estimated hours sum to 130-156 total (per SC-006)

### Example Validation Plugin

```javascript
// plugins/validate-data/index.js
module.exports = function (context, options) {
  return {
    name: 'validate-data-plugin',
    async contentLoaded({content, actions}) {
      const docs = content.loadedVersions[0].docs;

      // Validate prerequisites
      const prerequisiteErrors = [];
      docs.forEach(doc => {
        const {frontMatter, id} = doc;
        if (frontMatter.prerequisites) {
          frontMatter.prerequisites.forEach(prereq => {
            const prereqDoc = docs.find(d => d.id === prereq);
            if (!prereqDoc) {
              prerequisiteErrors.push(`${id}: Missing prerequisite ${prereq}`);
            } else if (prereqDoc.frontMatter.week >= frontMatter.week) {
              prerequisiteErrors.push(`${id}: Prerequisite ${prereq} comes after current chapter`);
            }
          });
        }
      });

      if (prerequisiteErrors.length > 0) {
        throw new Error(`Prerequisite validation failed:\n${prerequisiteErrors.join('\n')}`);
      }

      // Validate total estimated time
      const totalHours = docs.reduce((sum, doc) => sum + (doc.frontMatter.estimated_time || 0), 0);
      if (totalHours < 130 || totalHours > 156) {
        console.warn(`Total estimated time is ${totalHours} hours (expected 130-156)`);
      }

      console.log(`✅ Data validation passed: ${docs.length} chapters, ${totalHours} total hours`);
    },
  };
};
```

## Summary

This data model defines:
- **7 core entities:** Part, Module, Chapter, Assessment, Rubric, Glossary Entry, Hardware Configuration
- **4 component interfaces:** ModuleCard, GlossarySearch, PrerequisiteGraph, QuickLinksPanel
- **3 data file contracts:** glossary.json, modules.json, sidebars.ts
- **Build-time validation:** Prerequisites, metadata schema, totals

All entities and contracts support the functional requirements (FR-001 through FR-013) and success criteria (SC-001 through SC-010) defined in the specification.

---

**Status:** Complete
**Version:** 1.0.0
**Last Updated:** 2025-12-09
