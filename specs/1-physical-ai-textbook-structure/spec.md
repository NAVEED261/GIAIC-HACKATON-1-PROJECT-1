---
feature: Physical AI Textbook Structure
version: 1.0.0
status: Draft
created: 2025-12-09
updated: 2025-12-09
author: System
---

# Feature Specification: Physical AI Textbook Structure

## Overview

This specification defines the complete organizational structure, navigation system, and content framework for a 13-week Physical AI course textbook. The textbook must support industry practitioners learning robotics through a modular progression (ROS 2 ’ Digital Twin ’ Isaac Sim ’ VLA Integration), culminating in an autonomous humanoid capstone project.

The structure enables multiple access patterns (by week, by module, by topic), supports incremental content delivery, provides clear prerequisite tracking, and includes comprehensive reference materials and assessment guides.

**Why it's needed:** Industry practitioners transitioning to Physical AI need a structured, navigable learning path that balances theoretical foundations with hands-on implementation across modern robotics tools and frameworks.

## Target Audience

**Primary:** Industry practitioners with programming knowledge (Python) seeking to transition into Physical AI and humanoid robotics

**Secondary:**
- University instructors teaching semester/quarter Physical AI courses
- Self-directed learners following the 13-week curriculum
- Teams implementing autonomous humanoid systems

**Assumed Background:**
- Programming proficiency (Python)
- Basic understanding of AI/ML concepts
- Familiarity with command-line tools
- 10-12 hours/week time commitment

## User Scenarios & Testing

### User Story 1: Navigate Complete Course Structure (Priority: P1)

**As** an industry practitioner learning Physical AI
**I need** to see the complete 13-week course structure organized by modules and weeks
**So that** I can plan my learning journey and understand prerequisite relationships between topics

**Why this priority:** This is foundational - students cannot effectively use the textbook without understanding its organization and learning progression.

**Independent Test:** Can be fully tested by viewing the table of contents, confirming all 13 weeks are represented, all 4 modules are clearly delineated, and prerequisite chains are visible.

**Acceptance Scenarios:**

1. **Dashboard Overview**
   - **Given** I am a new student starting the course
   - **When** I open the textbook homepage
   - **Then** I see a dashboard with 4 module cards displaying titles and week ranges (Weeks 1-2, 3-5, 6-7, 8-10, 11-12, 13), plus a quick links sidebar with hardware setup, assessments, and glossary

2. **Course Progression View**
   - **Given** I want to understand course progression
   - **When** I view the table of contents
   - **Then** I can see which chapters belong to which weeks and modules

3. **Module Content Grouping**
   - **Given** I am on Week 5
   - **When** I navigate to Module 1 content
   - **Then** I can see all chapters for Weeks 3-5 grouped together

4. **Prerequisite Visibility**
   - **Given** I need to review prerequisites
   - **When** I open any chapter
   - **Then** I see clearly stated prerequisite weeks/chapters

### User Story 2: Access Foundational Setup Documentation (Priority: P1)

**As** an industry practitioner
**I need** immediate access to hardware setup guides, environment configuration, and glossary references before starting course content
**So that** I can prepare my development environment and understand terminology

**Why this priority:** Students cannot begin hands-on work without proper hardware and software setup. This blocks all other learning.

**Independent Test:** Can be fully tested by accessing hardware setup guide, verifying all 3 hardware options are documented (Digital Twin Workstation, Edge Kit, Cloud setup), and confirming glossary is searchable.

**Acceptance Scenarios:**

1. **Hardware Setup Access**
   - **Given** I am setting up my learning environment
   - **When** I navigate to the setup section
   - **Then** I see detailed guides for all 3 hardware options (Workstation, Jetson Kit, Cloud)

2. **Glossary Search**
   - **Given** I encounter unfamiliar robotics terminology
   - **When** I use the dedicated glossary search component
   - **Then** I get instant term lookup with definitions and links to relevant chapters

3. **Software Installation**
   - **Given** I need to install ROS 2 and Isaac Sim
   - **When** I follow the setup guide
   - **Then** I have step-by-step instructions with version specifications

4. **Hardware Comparison**
   - **Given** I have budget constraints
   - **When** I review hardware options
   - **Then** I can compare costs and capabilities to make an informed decision

### User Story 3: Follow Module-Based Learning Path (Priority: P2)

**As** an industry practitioner
**I need** to progress through each module (ROS 2 ’ Digital Twin ’ Isaac ’ VLA) sequentially with clear module objectives and capstone integration points
**So that** I understand how each module builds toward the final autonomous humanoid project

**Why this priority:** Module structure provides the pedagogical framework. Students need to see how topics connect and build toward the capstone.

**Independent Test:** Can be fully tested by navigating through each module, verifying module learning outcomes are stated, and confirming capstone integration points are documented.

**Acceptance Scenarios:**

1. **Module Integration Understanding**
   - **Given** I completed Module 1 (ROS 2)
   - **When** I start Module 2 (Digital Twin)
   - **Then** I see how ROS 2 concepts are applied in simulation

2. **Learning Outcomes Clarity**
   - **Given** I am working on Module 3 (Isaac)
   - **When** I review the module overview
   - **Then** I see clear learning outcomes and how this contributes to the capstone project

3. **Study Planning**
   - **Given** I am planning my study schedule
   - **When** I view module summaries
   - **Then** I can estimate time commitment for each module (2-3 weeks per module)

4. **Capstone Integration**
   - **Given** I completed all modules
   - **When** I reach Module 4 final section
   - **Then** I have a clear integration guide for the capstone project

### User Story 4: Access Assessment and Project Guidelines (Priority: P2)

**As** an industry practitioner
**I need** to access assessment criteria, project rubrics, and capstone requirements throughout the course
**So that** I can self-assess my progress and understand evaluation expectations

**Why this priority:** Clear assessment criteria helps students focus effort and validates learning. This supports self-directed learning for practitioners.

**Independent Test:** Can be fully tested by locating all 4 assessment types (ROS 2 package, Gazebo simulation, Isaac perception, Capstone) and verifying each has a detailed rubric.

**Acceptance Scenarios:**

1. **Assessment Access**
   - **Given** I completed a module
   - **When** I navigate to the assessments section
   - **Then** I see the relevant project requirements and evaluation rubric

2. **Rubric Clarity**
   - **Given** I am working on the ROS 2 package project
   - **When** I review the rubric
   - **Then** I understand exactly what constitutes passing, good, and excellent work

3. **Capstone Architecture**
   - **Given** I am planning the capstone
   - **When** I access the capstone guide
   - **Then** I see the 5-step architecture (voice ’ plan ’ navigate ’ perceive ’ manipulate) clearly documented

4. **Self-Assessment**
   - **Given** I want to check my progress
   - **When** I review assessment checklists
   - **Then** I can self-evaluate against measurable criteria

### User Story 5: Reference Quick Guides and Troubleshooting (Priority: P3)

**As** an industry practitioner
**I need** quick-reference guides for ROS 2 commands, Isaac Sim operations, and common troubleshooting scenarios
**So that** I can resolve issues without searching through full chapters

**Why this priority:** Efficiency improvement for experienced users. Reduces friction but not critical for initial learning.

**Independent Test:** Can be fully tested by accessing quick reference section, searching for a common error, and verifying troubleshooting steps are provided.

**Acceptance Scenarios:**

1. **Command Reference**
   - **Given** I forgot a ROS 2 command
   - **When** I access the ROS 2 quick reference
   - **Then** I see a command cheat sheet with examples

2. **Error Resolution**
   - **Given** I encounter an Isaac Sim error
   - **When** I search the troubleshooting guide
   - **Then** I find common errors and solutions

3. **Notation Review**
   - **Given** I need to review notation
   - **When** I access the notation guide
   - **Then** I see mathematical symbols and conventions used throughout the book

4. **External Resources**
   - **Given** I need external resources
   - **When** I visit the resources section
   - **Then** I find links to official documentation, papers, and community forums

### Edge Cases

1. **Skipping Modules**
   - **What happens when** a student wants to skip directly to Module 3 (Isaac) without completing Modules 1-2 (ROS 2, Gazebo)?
   - **Resolution:** Prerequisites section clearly warns about required knowledge; student can attempt but may struggle. The prerequisite dependency graph is visible.

2. **Alternative Hardware**
   - **What happens when** the structure handles students using alternative hardware (e.g., non-NVIDIA GPUs)?
   - **Resolution:** Hardware setup guide includes a "Limitations and Alternatives" section documenting what works and what doesn't.

3. **Cloud-Only Access**
   - **What happens if** a student only has access to cloud infrastructure?
   - **Resolution:** Cloud-native setup guide is a first-class option with specific chapters on cloud deployment.

4. **Course Format Customization**
   - **How do** instructors customize chapter order for different course formats?
   - **Resolution:** Book structure is modular; instructors section explains how to reorder or skip chapters with prerequisite warnings.

## Requirements

### Functional Requirements

**FR-001:** Book MUST organize content into 4 distinct modules aligned with course structure:
- Module 1 (ROS 2 - Weeks 3-5)
- Module 2 (Digital Twin - Weeks 6-7)
- Module 3 (NVIDIA Isaac - Weeks 8-10)
- Module 4 (VLA & Humanoids - Weeks 11-13)

**FR-002:** Book MUST include a dedicated "Introduction" section covering Weeks 1-2 (Physical AI Foundations) before Module 1

**FR-003:** Book MUST provide 3 hardware setup paths:
1. Digital Twin Workstation (RTX GPU + Ubuntu 22.04)
2. Physical AI Edge Kit (Jetson Orin Nano)
3. Cloud-Native Setup (AWS/Azure)

**FR-004:** Each module MUST state clear learning outcomes that map to course learning outcomes

**FR-005:** Book MUST include a "Capstone Project Guide" section detailing the autonomous humanoid project architecture with 5 components: voice ’ plan ’ navigate ’ perceive ’ manipulate

**FR-006:** Book MUST provide 4 assessment guides corresponding to course assessments:
1. ROS 2 package development
2. Gazebo simulation
3. Isaac perception pipeline
4. Capstone project

**FR-007:** Book MUST include reference materials:
- Glossary (robotics terminology with dedicated search component)
- Notation Guide (mathematical symbols)
- ROS 2 Quick Reference
- Troubleshooting Guide

**FR-007a:** Search functionality MUST use a hybrid approach:
- Algolia DocSearch for main content with custom metadata indexing (week, module, topic)
- Dedicated glossary search component for instant term lookup

**FR-008:** Navigation structure MUST use a single sidebar with nested collapsible categories organized by modules, with cross-references enabling access by:
- Week (1-13)
- Module (1-4)
- Topic (kinematics, perception, etc.)

**FR-009:** Each chapter MUST declare prerequisites (specific prior weeks/chapters or external knowledge)

**FR-010:** Book structure MUST support incremental content delivery (Week 1-2 content can be published independently of later weeks)

**FR-011:** Each chapter MUST include frontmatter metadata:
- estimated_time (hours)
- week (1-13)
- module (1-4)
- prerequisites (array of prior chapters/weeks)
- learning_objectives (array)
- sidebar_label
- Optional: assessment_type, difficulty_level, capstone_component

Table of contents MUST display estimated time to help students plan study schedules.

**FR-012:** Book MUST include an "Instructors Guide" section explaining:
- How to customize chapter order
- Recommended lab exercises
- Adaptation for different course formats (semester vs. quarter)

**FR-013:** Homepage MUST use a dashboard-style layout featuring:
1. Grid of 4 module cards showing title, week range, and learning outcomes
2. Quick links sidebar for hardware setup, assessments, and glossary
3. Recent updates section for content changes

## Key Entities

### Module
Represents a major course section (4 total: ROS 2, Digital Twin, Isaac, VLA).

**Attributes:**
- Title
- Week range
- Learning outcomes (array)
- Chapters (array of chapter references)
- Integration points with capstone

**Relationships:**
- Contains multiple Chapters
- Builds toward Capstone Project

### Chapter
Represents a single topic within a module.

**Metadata:**
- estimated_time (hours)
- week (number 1-13)
- module (number 1-4)
- prerequisites (array of prior chapters/weeks)
- learning_objectives (array)
- sidebar_label (string)
- assessment_type (optional)
- difficulty_level (optional)
- capstone_component (optional)

**Content:**
- Content sections
- Code examples (embedded)
- Exercises
- References

**Relationships:**
- Belongs to one Module
- Maps to 1-2 weeks of course content
- References Prerequisites (other Chapters)
- May contribute to Assessment

### Part
High-level grouping of content.

**Types:**
- Introduction (Weeks 1-2)
- Foundational Setup
- Module 1 (ROS 2)
- Module 2 (Digital Twin)
- Module 3 (Isaac)
- Module 4 (VLA)
- Capstone Guide
- Assessments
- References

**Relationships:**
- Contains multiple Modules or reference sections

### Hardware Configuration
Represents one of three setup paths.

**Attributes:**
- Hardware requirements (list)
- Software installation steps (ordered list)
- Cost estimate (USD range)
- Limitations (list)

**Types:**
1. Digital Twin Workstation
2. Physical AI Edge Kit
3. Cloud-Native Setup

### Assessment
Represents a project or evaluation point.

**Attributes:**
- Requirements (list)
- Rubric (structured evaluation criteria with 3+ levels)
- Evaluation criteria (checklist)
- Submission guidelines

**Types:**
1. ROS 2 Package Development
2. Gazebo Simulation
3. Isaac Perception Pipeline
4. Capstone Project

**Relationships:**
- References multiple Chapters
- Uses Rubric for evaluation

### Reference Material
Includes support resources.

**Types:**
- Glossary Entry (term + definition + chapter links)
- Notation Definition (symbol + meaning)
- Quick Reference Command (command + example + usage)
- Troubleshooting Solution (error + solution + related chapters)
- External Resource (link + description)

**Relationships:**
- Glossary Entries link to Chapters
- Troubleshooting Solutions reference Chapters

## Success Criteria

This feature is complete when the following measurable outcomes are achieved:

**SC-001:** Students can locate any week's content within 2 clicks from the homepage (navigation efficiency test)

**SC-002:** 95% of prerequisite references are correctly linked (no broken internal links to prior chapters)

**SC-003:** Each of the 4 modules has clearly stated learning outcomes that align with course learning outcomes (verifiable by side-by-side comparison)

**SC-004:** All 3 hardware setup paths are documented with complete step-by-step instructions (verifiable by successfully following setup guide)

**SC-005:** Students can identify which chapters contribute to each capstone project component (voice, plan, navigate, perceive, manipulate) within 5 minutes of reviewing the capstone guide

**SC-006:** Table of contents shows estimated time commitment for each chapter, enabling students to plan a 13-week study schedule totaling 130-156 hours (10-12 hours/week × 13 weeks)

**SC-007:** Glossary contains at least 100 robotics terms with definitions and chapter cross-references, accessible via dedicated search component providing instant lookup (response time < 2 seconds)

**SC-008:** Each assessment has a detailed rubric with at least 3 evaluation levels (e.g., needs improvement, proficient, excellent)

**SC-009:** Book structure supports publishing Week 1-2 content independently as a functional mini-textbook (Introduction + Setup + Glossary can be deployed and navigated without later content)

**SC-010:** Instructors can customize chapter order by following the prerequisite dependency graph without breaking learning progression (dependency graph is clear and complete)

## Scope

### In Scope

- Complete textbook organizational structure for 13 weeks of content
- 4-module progression: ROS 2 (Weeks 3-5), Digital Twin (Weeks 6-7), Isaac (Weeks 8-10), VLA & Humanoids (Weeks 11-13)
- Introduction section (Weeks 1-2) covering Physical AI foundations
- 3 hardware setup paths with detailed documentation
- Navigation system supporting access by week, module, and topic
- Prerequisite dependency tracking across all chapters
- Capstone project guide detailing 5-component architecture (voice ’ plan ’ navigate ’ perceive ’ manipulate)
- 4 assessment guides with rubrics
- Reference materials: Glossary (100+ terms), Notation Guide, ROS 2 Quick Reference, Troubleshooting Guide
- Instructor customization guidance
- Dashboard-style homepage with module cards and quick links
- Hybrid search functionality (Algolia DocSearch + dedicated glossary search)
- Metadata schema for chapters (estimated_time, week, module, prerequisites, learning_objectives)
- Support for incremental content delivery (Week 1-2 publishable independently)

### Out of Scope

- Actual chapter content authoring (covered in separate chapter-specific specifications)
- Code example repository structure (code embedded in markdown per spec requirements)
- Video content or interactive multimedia
- Embedded interactive simulations
- Automated grading systems or LMS integration
- Student progress tracking features
- Multi-language translation (English only for v1.0)
- Mobile native applications (web-responsive only)
- PDF export functionality (future enhancement)
- Community forum or discussion features

## Assumptions

- Students are industry practitioners with programming knowledge (Python) as stated in course requirements
- Course duration is fixed at 13 weeks with 10-12 hours/week commitment (130-156 total hours)
- Capstone project is the culminating assessment and all modules build toward it
- Hardware requirements follow NVIDIA Isaac Sim specifications (RTX GPU for workstation path)
- Content will be delivered via Docusaurus static site deployed to GitHub Pages
- ROS 2 version is Humble or Iron (Ubuntu 22.04 compatible)
- Isaac Sim is the primary simulation environment for Modules 3-4
- Students have access to at least one of the three hardware configurations
- Assessment rubrics follow academic standards for technical courses
- Book will be continuously updated as hardware/software evolves
- English is the primary language (no translation for v1.0)
- Internet access is available for cloud-based options and resource links
- Students have basic Git and command-line proficiency
- Docusaurus v3 is the static site generator
- GitHub provides version control and Pages hosting

## Dependencies

### Technical Dependencies

**Constitution Principles:**
The book structure must comply with constitution principles, especially:
- Principle III: Consistency & Standards (chapter structure template)
- Principle IV: Docusaurus Structure & Quality (navigation, metadata)
- Principle VII: Deployment & Publishing Standards (build gates, performance)

**Course Syllabus:**
Official 13-week syllabus defines week-by-week topics and must be reflected in book structure (external dependency on curriculum design)

**Hardware Specifications:**
NVIDIA Isaac Sim requirements dictate hardware setup documentation (RTX GPU, Ubuntu 22.04 compatibility)

**Docusaurus Framework:**
Book structure must be compatible with Docusaurus v3 sidebar and navigation capabilities

**External Tools:**
- ROS 2 (Humble/Iron versions)
- Gazebo Classic or Gazebo (version TBD)
- Unity ML-Agents (for Digital Twin module)
- Isaac Sim (latest stable version)
- NVIDIA Jetson Orin Nano SDK
- Cloud platforms (AWS/Azure) APIs and services

**Search Integration:**
- Algolia DocSearch service availability and API
- Custom glossary search component implementation

### Content Dependencies

**Templates:**
- Chapter content template (for consistent chapter structure)
- Metadata schema (for frontmatter)
- Rubric template (for assessments)

**Reference Data:**
- Glossary term compilation (100+ terms)
- ROS 2 command reference data
- Troubleshooting scenarios database

**Course Materials:**
- Learning outcomes mapping to industry standards
- Prerequisite dependency graph
- Capstone project architecture specification

## Clarifications

### Session 2025-12-09

**Q1: Docusaurus sidebar organization pattern for supporting multiple access patterns (by week, by module, by topic)?**
**A:** Single sidebar with nested collapsible categories (Intro ’ Module 1 ’ Module 2, etc.) with cross-references for topics.

**Q2: Chapter frontmatter metadata requirements beyond Docusaurus defaults?**
**A:** Custom metadata with assessment tracking including estimated_time, week, module, prerequisites (array), learning_objectives (array), sidebar_label, assessment_type, difficulty_level, capstone_component

**Q3: Homepage layout and course overview presentation?**
**A:** Dashboard-style with module cards + quick links sidebar (hardware setup, assessments, glossary) + recent updates section

**Q4: Search functionality configuration for content and glossary?**
**A:** Hybrid approach using Algolia DocSearch for main content with custom metadata indexing, plus dedicated glossary search component for term lookup

**Q5: Code example repository structure and hosting?**
**A:** Embedded code snippets only in markdown files, no separate example file repository (simplifies maintenance, though deviates from Constitution Principle V)

## Constitution Compliance Notes

### Deviation from Constitution Principle V (Code Example Quality)

**Constitution requires:**
Repository structure: /examples/[chapter-name]/[example-name]/ with README explaining purpose and usage

**This spec chooses:**
Embedded code snippets only in markdown files (no separate example repository)

**Justification:**
For a textbook focused on educational content delivery, embedded snippets provide simpler maintenance and ensure code-content synchronization. Individual chapter specs will define complete, tested code examples embedded directly in markdown. Students can copy-paste examples for hands-on practice. This reduces repository complexity and aligns with the book structure's goal of being a readable, navigable textbook rather than a code distribution platform.

**Alternative considered:**
Monorepo with /examples/ directory rejected due to increased maintenance burden for 15-20 chapters with multiple examples each.

**Mitigation:**
Each embedded code example will still follow Constitution Principle V requirements for:
- Completeness (full runnable code)
- Documentation (via surrounding text and inline comments)
- Testing (validated before publication)
- Version specifications (dependencies noted in comments)

## Notes

- Book structure should support phased rollout: Introduction & Setup ’ Module 1 ’ Module 2 ’ Module 3 ’ Module 4 ’ Capstone
- Each module can be developed in parallel once overall structure is defined
- Hardware setup guide should be continuously updated as new hardware options become available
- Consider adding a "Learning Pathways" visualization showing how chapters connect to capstone components (future enhancement)
- Instructors guide should include tips for adapting to semester (15-week) vs. quarter (10-week) formats
- Assessment rubrics should reference specific chapters/exercises to make grading objective
- Code examples will be embedded as fenced code blocks with language specification, comments, and dependency notes per Constitution Principle V
- Glossary search component may require custom React component development in Docusaurus
- Sidebar nested structure may require Docusaurus plugin or custom configuration
- Metadata schema should be validated during build process to ensure consistency
- Consider automated link checking in CI/CD pipeline to maintain 95% prerequisite link success rate
- Homepage dashboard may require custom Docusaurus landing page component
- Algolia DocSearch integration requires Algolia account and configuration
- Recent updates section could be automated via Git commit history or manual changelog

---

**Status:** Draft
**Version:** 1.0.0
**Last Updated:** 2025-12-09
