# Specifications

This directory contains feature/chapter specifications, plans, and tasks for the book project.

## Directory Structure

Each feature or chapter has its own directory:

```
specs/
├── <feature-name>/
│   ├── spec.md           # Requirements and learning objectives
│   ├── plan.md           # Implementation plan and architecture
│   ├── tasks.md          # Granular task breakdown
│   └── prompts.md        # AI prompts used (optional)
└── README.md
```

## Spec-Driven Development Workflow

### 1. Specification Phase (`/sp.specify`)

Create `spec.md` to define:
- Learning objectives
- Target audience
- Prerequisites
- Scope (in/out of scope)
- Key concepts
- Content structure
- Code examples needed
- Diagrams needed
- Exercises
- Success criteria

### 2. Planning Phase (`/sp.plan`)

Create `plan.md` to define:
- Architecture and design decisions
- Content organization
- Technical implementation details
- Visual assets plan
- Quality and testing strategy
- Risk analysis
- Complexity justifications

### 3. Task Breakdown (`/sp.tasks`)

Create `tasks.md` to define:
- Actionable tasks with acceptance criteria
- Dependencies between tasks
- Test cases for each task
- Complexity estimates
- Progress tracking

### 4. Implementation

Execute tasks following the plan while adhering to constitution principles.

### 5. Review

Technical review, peer review, and quality gates validation.

### 6. Publishing

Build validation → PR → Merge → Deploy

## Feature Naming

Features/chapters use kebab-case naming:

```
getting-started
ai-prompting-basics
docusaurus-customization
github-pages-deployment
```

## Creating a New Feature Spec

Use the command:

```bash
/sp.specify <feature-name>
```

Or manually:

```bash
mkdir -p specs/<feature-name>
cp .specify/templates/spec-template.md specs/<feature-name>/spec.md
cp .specify/templates/plan-template.md specs/<feature-name>/plan.md
cp .specify/templates/tasks-template.md specs/<feature-name>/tasks.md
```

## Spec Status Lifecycle

Specs progress through these states:

1. **Draft** - Initial creation
2. **Review** - Under review
3. **Approved** - Ready for planning
4. **Planning** - Creating plan.md
5. **Ready** - Plan approved, tasks defined
6. **In Progress** - Implementation ongoing
7. **Complete** - Published and deployed

## Best Practices

### Specification
- Define clear, measurable learning objectives
- Specify prerequisites explicitly
- Identify all required code examples and diagrams upfront
- Include diverse exercises (conceptual, hands-on, project-based)

### Planning
- Document architectural decisions (create ADRs for significant ones)
- Justify any deviations from constitution
- Identify risks and mitigation strategies
- Plan for quality assurance

### Tasks
- Break down into small, testable units
- Define clear acceptance criteria
- Include test cases where applicable
- Track dependencies

### Implementation
- Follow constitution principles
- Reference spec and plan documents
- Update tasks as you progress
- Create PHRs for AI-assisted work

## Template Placeholders

All templates use `{{PLACEHOLDER}}` format:

- `{{FEATURE_NAME}}` - Name of the feature/chapter
- `{{DATE_ISO}}` - ISO date (YYYY-MM-DD)
- `{{VERSION}}` - Version number
- `{{STATUS}}` - Current status
- `{{AUTHOR}}` - Author name

Replace these when creating new specs.

## Linking

Specs should link to:
- Related ADRs (architectural decisions)
- Related PHRs (AI collaboration records)
- Related PRs (implementation pull requests)
- Related chapters (dependencies)

## Example Structure

```
specs/
├── introduction/
│   ├── spec.md
│   ├── plan.md
│   └── tasks.md
├── docusaurus-basics/
│   ├── spec.md
│   ├── plan.md
│   ├── tasks.md
│   └── prompts.md
└── advanced-customization/
    ├── spec.md
    ├── plan.md
    └── tasks.md
```

---

**Last Updated:** 2025-12-09
