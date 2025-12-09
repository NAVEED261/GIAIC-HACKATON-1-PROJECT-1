# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Physical AI Textbook Project** - An AI/Spec-Driven book creation system using Docusaurus v3 and GitHub Pages to build a comprehensive 13-week Physical AI course textbook covering ROS 2, Digital Twin simulation, NVIDIA Isaac Sim, and VLA integration for humanoid robotics.

**Development Methodology:** Spec-Driven Development (SDD) using SpecKit Plus workflow
- All features begin with `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement`
- Every user interaction generates a Prompt History Record (PHR)
- Architectural decisions require explicit ADR documentation

## SpecKit Plus Workflow Commands

**Core Workflow (Execute in Order):**

1. **`/sp.specify`** - Create feature specification with user stories, acceptance criteria, and success metrics
   - Generates: `specs/<feature-name>/spec.md`
   - Creates feature branch: `<feature-number>-<feature-name>`
   - Validates against constitution principles

2. **`/sp.plan`** - Generate implementation plan with architectural decisions
   - Generates: `specs/<feature-name>/plan.md`, `research/research.md`, `data-model.md`, `contracts/*.json`
   - Documents technology stack, component architecture, and build pipeline
   - Creates prerequisite dependency graphs

3. **`/sp.tasks`** - Break down implementation into 75+ granular tasks organized by user story
   - Generates: `specs/<feature-name>/tasks.md`
   - Tasks follow strict format: `- [ ] T### [P] [US#] Description with file path`
   - Includes parallel execution opportunities and MVP scope definition

4. **`/sp.implement`** - Execute tasks from tasks.md (not yet scaffolded in this project)

**Supporting Commands:**

- **`/sp.constitution`** - Create or update project constitution with core principles
- **`/sp.adr <title>`** - Document architectural decision (must wait for user consent)
- **`/sp.clarify`** - Ask targeted clarification questions during specification
- **`/sp.analyze`** - Cross-artifact consistency analysis (spec → plan → tasks)
- **`/sp.checklist`** - Generate custom validation checklist
- **`/sp.phr`** - Manually create Prompt History Record
- **`/sp.git.commit_pr`** - Autonomous git commit and PR creation workflow

## Critical PHR (Prompt History Record) Requirements

**MANDATORY:** Create PHR after every user interaction involving work execution.

**PHR Routing (all under `history/prompts/`):**
- Constitution work → `history/prompts/constitution/`
- Feature-specific work → `history/prompts/<feature-name>/`
- General work → `history/prompts/general/`

**PHR Stages:** constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

**Creation Process:**
1. Read template: `.specify/templates/phr-template.prompt.md`
2. Allocate ID (increment from existing PHRs in target directory)
3. Fill ALL placeholders (ID, TITLE, STAGE, DATE_ISO, MODEL, FEATURE, BRANCH, USER, COMMAND, LABELS, FILES_YAML, TESTS_YAML, PROMPT_TEXT, RESPONSE_TEXT)
4. Validate: No placeholders remain, file exists at correct path, prompt text not truncated
5. Report: Print ID, path, stage, title

**File naming:**
- Constitution: `<ID>-<slug>.constitution.prompt.md`
- Feature: `<ID>-<slug>.<stage>.prompt.md`
- General: `<ID>-<slug>.general.prompt.md`

## Project Structure

```
├── .specify/                       # SpecKit Plus framework
│   ├── memory/
│   │   └── constitution.md        # 8 core principles (Content Accuracy, UI/UX, Deployment, etc.)
│   ├── templates/                 # Templates for spec, plan, tasks, PHR, ADR
│   └── scripts/                   # Bash/PowerShell automation scripts
├── specs/                         # Feature specifications
│   └── <feature-number>-<feature-name>/
│       ├── spec.md               # User stories, requirements, success criteria
│       ├── plan.md               # Architecture, decisions, component design
│       ├── tasks.md              # Checklist of implementation tasks
│       ├── data-model.md         # Entity relationships, TypeScript interfaces
│       ├── research/
│       │   └── research.md       # Technical research and decision rationale
│       ├── contracts/            # JSON schemas for validation
│       │   ├── chapter-frontmatter.schema.json
│       │   ├── glossary.schema.json
│       │   └── modules.schema.json
│       └── checklists/
│           └── requirements.md   # Validation checklist
├── history/
│   ├── prompts/                  # Prompt History Records
│   │   ├── constitution/
│   │   ├── <feature-name>/
│   │   └── general/
│   └── adr/                      # Architecture Decision Records
├── docs/                         # Placeholder for Docusaurus content
├── examples/                     # Placeholder for code examples
└── .claude/commands/             # Custom slash commands (11 total)
```

## Constitution Principles (Must Follow)

Located in `.specify/memory/constitution.md` - **READ THIS FILE** before making any changes.

**8 Core Principles:**

1. **Content Accuracy & Technical Rigor** - All code examples tested, versions specified, claims require evidence
2. **Educational Clarity & Accessibility** - Consistent terminology, visual aids, measurable learning objectives
3. **Consistency & Standards** - Uniform chapter structure, PEP 8 formatting, kebab-case file naming
4. **Docusaurus Structure & Quality** - Lighthouse ≥90, mobile responsive (375px-1440px), no broken links
5. **Code Example Quality** - Complete imports, dependency versions, descriptive variable names, error handling
6. **UI/UX Excellence** - 2-click max navigation, <2s glossary search, prerequisite chains visible
7. **Deployment & Publishing** - GitHub Actions CI/CD, quality gates (build, links, spell, Lighthouse, a11y)
8. **AI-Driven Content Standards** - Human review required, test AI code, verify facts, check bias

**Key Constraints:**
- Chapter frontmatter must include: `week` (1-13), `module` (1-4), `estimated_time`, `prerequisites`, `learning_objectives`
- File naming: kebab-case markdown (e.g., `ros2-fundamentals.md`)
- Image optimization: SVG preferred, PNG <200KB
- Code blocks: Always specify language (```python, ```bash)
- No broken internal links (enforced in CI/CD)

## Feature #1: Physical AI Textbook Structure

**Current State:** Planning complete, implementation pending

**Specification:** `specs/1-physical-ai-textbook-structure/spec.md`
- 5 user stories (P1: Navigation & Setup, P2: Modules & Assessments, P3: References)
- 13 functional requirements (FR-001 through FR-013)
- 10 success criteria (SC-001 through SC-010)

**Technology Stack:**
- Docusaurus v3 (static site generator)
- React 18 + TypeScript (custom components)
- Algolia DocSearch (content search)
- Fuse.js (glossary fuzzy search <2s)
- React Flow + dagre (prerequisite graphs)
- Node.js 18+

**Implementation Tasks:** `specs/1-physical-ai-textbook-structure/tasks.md`
- 75 tasks across 8 phases
- MVP: Tasks T001-T020 (User Story 1 only - navigation structure)
- 42 tasks marked `[P]` for parallel execution
- Task format: `- [ ] T### [P] [US#] Description with file path`

**Key Components to Implement:**
- `ModuleCard.tsx` - Dashboard module cards (4 modules × week ranges)
- `GlossarySearch.tsx` - Instant term lookup with Fuse.js
- `PrerequisiteGraph.tsx` - Dependency visualization with React Flow
- `QuickLinksPanel.tsx` - Fixed sidebar navigation

**Data Contracts:**
- Chapter frontmatter schema: `contracts/chapter-frontmatter.schema.json`
- Glossary data (100+ terms): `contracts/glossary.schema.json`
- Module metadata (4 modules): `contracts/modules.schema.json`

## Architectural Decisions

**When to Suggest ADR:**
Test for significance (ALL must be true):
- **Impact:** Long-term consequences (framework, data model, API, platform)
- **Alternatives:** Multiple viable options considered
- **Scope:** Cross-cutting, influences system design

If true, suggest: `📋 Architectural decision detected: <brief>. Document? Run /sp.adr <title>`

**Never auto-create ADRs** - always wait for user consent.

**Existing Decisions (from plan.md):**
1. Docusaurus v3 as static site generator (vs VuePress, GitBook, Next.js)
2. Single sidebar with nested categories (vs multiple sidebars, hybrid nav)
3. Custom frontmatter metadata (vs Docusaurus defaults only)
4. Hybrid search (Algolia + Fuse.js vs Algolia-only)
5. Custom React homepage (vs standard Docusaurus landing)

## Git Workflow

**Branch Naming:** `<feature-number>-<feature-name>` (e.g., `1-physical-ai-textbook-structure`)

**Commit Message Format:**
```
<Summary line>

<Detailed description>
- Bullet points for key changes
- Reference feature numbers

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude <noreply@anthropic.com>
```

**PR Creation:** Use `/sp.git.commit_pr` command for autonomous workflow

**Main Branch:** `master` (not `main`)

## Development Guidelines

**Planning First:**
- Never implement without spec.md → plan.md → tasks.md
- Clarify ambiguous requirements with `/sp.clarify` before proceeding
- Document all architectural decisions with `/sp.adr`

**Quality Gates (Enforced in CI/CD):**
1. Build succeeds without warnings
2. All internal links resolve (no 404s)
3. Spell check passes (robotics dictionary)
4. Lighthouse scores ≥90 (performance, accessibility, SEO)
5. WCAG AA accessibility compliance

**Task Execution:**
- Follow checklist format strictly: `- [ ] T### [P] [US#] Description with file path`
- Mark tasks with `[P]` if parallelizable
- Label user story tasks with `[US1]`, `[US2]`, etc.
- Include exact file paths in task descriptions
- Independent test criteria per user story

**Validation:**
- Chapter metadata: week 1-13, module 1-4, estimated_time >0
- Prerequisite references must exist (no broken chapter links)
- Total course hours: 130-156 hours (10-12 hrs/week × 13 weeks)
- Glossary: minimum 100 terms with chapter cross-references

## Human-in-the-Loop

**Always Ask User When:**
1. Requirements are ambiguous (ask 2-3 targeted clarifying questions)
2. Discovering dependencies not in spec (surface and ask for prioritization)
3. Multiple valid approaches exist (present options with tradeoffs)
4. Completing major milestones (summarize and confirm next steps)

**Never Assume:**
- APIs, data schemas, or contracts (ask for clarification)
- Secrets/tokens (use `.env` and document in setup guide)
- User intent (confirm before proceeding with significant work)

## Current Status

**Completed:**
- ✅ Constitution defined (8 core principles)
- ✅ Feature #1 specification (5 user stories, 13 FRs, 10 SCs)
- ✅ Implementation plan (5 architectural decisions, component specs)
- ✅ Research document (7 technical topics)
- ✅ Data model (7 entities, TypeScript interfaces)
- ✅ JSON schema contracts (chapter, glossary, modules)
- ✅ Implementation tasks (75 tasks across 8 phases)
- ✅ 3 PHRs documenting planning workflow
- ✅ All artifacts merged to `master` branch

**Next Step:** Run `/sp.implement` to execute tasks T001-T075, starting with MVP (T001-T020)

**Active Branch:** `master`
**Feature Branch:** `1-physical-ai-textbook-structure` (available for checkout)

## Code Standards Reference

**File Naming:**
- Markdown: `kebab-case.md` (e.g., `ros2-fundamentals.md`)
- React components: `PascalCase.tsx` (e.g., `ModuleCard.tsx`)
- Images: `{chapter-slug}-{description}.{svg|png}`
- JSON schemas: `entity-name.schema.json`

**Frontmatter Template (All Chapters):**
```yaml
---
title: "Chapter Title"
description: "SEO description (20-160 chars)"
sidebar_label: "Short Label"
sidebar_position: 1
estimated_time: 3  # hours
week: 3  # 1-13
module: 1  # 1-4
prerequisites: ["chapter-id-1", "chapter-id-2"]
learning_objectives:
  - "Measurable objective 1"
  - "Measurable objective 2"
assessment_type: "ros2-package"  # optional
difficulty_level: "intermediate"  # optional
capstone_component: "navigation"  # optional
---
```

**Code Block Format:**
````markdown
```python
# Dependencies: rospy==1.15.0, numpy>=1.21
import rospy
import numpy as np

# Explain WHY, not WHAT
def calculate_joint_angles(target_position):
    """Calculate inverse kinematics for target position."""
    # Error handling for realistic scenarios
    if not rospy.is_shutdown():
        return inverse_kinematics(target_position)
    raise rospy.ROSInterruptException("ROS shutdown")
```
````

## Troubleshooting

**Constitution file corrupt (`/SP` only):**
- Already fixed - file restored with 8 core principles
- If corrupted again, restore from git: `git show HEAD:.specify/memory/constitution.md`

**PowerShell scripts not available (pwsh: command not found):**
- Use bash equivalents in `.specify/scripts/bash/`
- Manually determine feature paths from git branch name

**PHR creation fails:**
- Read template: `.specify/templates/phr-template.prompt.md`
- Manually create PHR with Write tool (agent-native flow)
- Validate all placeholders filled before reporting

---

**Project Constitution:** `.specify/memory/constitution.md`
**Current Feature Spec:** `specs/1-physical-ai-textbook-structure/spec.md`
**Implementation Tasks:** `specs/1-physical-ai-textbook-structure/tasks.md`
