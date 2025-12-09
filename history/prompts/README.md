# Prompt History Records (PHRs)

This directory contains Prompt History Records (PHRs) that document AI-assisted work throughout the project.

## Directory Structure

```
prompts/
├── constitution/          # PHRs related to constitution creation/updates
├── general/              # General PHRs not tied to specific features
└── <feature-name>/       # Feature-specific PHRs
```

## PHR Purpose

Prompt History Records serve several purposes:

1. **Traceability:** Track what was requested and what was delivered
2. **Learning:** Understand how AI assistance was used effectively
3. **Context:** Provide context for future work on related features
4. **Documentation:** Maintain a record of decision-making process

## PHR Lifecycle

1. **Creation:** PHR created automatically after AI-assisted work
2. **Review:** PHRs reviewed periodically for insights
3. **Archival:** Old PHRs retained for historical reference

## PHR Naming Convention

```
<ID>-<slug>.<stage>.prompt.md
```

Examples:
- `001-create-constitution.constitution.prompt.md`
- `042-implement-sidebar.implementation.prompt.md`
- `099-fix-navigation-bug.bugfix.prompt.md`

## Stages

- `constitution` - Constitution-related work
- `spec` - Specification creation
- `plan` - Planning and architecture
- `tasks` - Task breakdown
- `implementation` - Code implementation
- `review` - Review and feedback
- `bugfix` - Bug fixes
- `refactor` - Code refactoring
- `general` - General or miscellaneous

## PHR Template

PHRs follow the template defined in `.specify/templates/phr-template.prompt.md`.

Required fields:
- ID, title, stage, date
- User, model, command
- Prompt text (full, not truncated)
- Response summary
- Files modified/created
- Outcome

## Best Practices

1. **Complete prompts:** Always include full prompt text
2. **Accurate responses:** Summarize key assistant outputs
3. **File tracking:** List all files created or modified
4. **Outcome clarity:** Describe what was accomplished
5. **Linking:** Link to related specs, ADRs, PRs

## Searching PHRs

Find PHRs by:
- **Stage:** Search within stage-specific directories
- **Feature:** Search within feature directories
- **Content:** Use grep to search prompt/response text
- **Date:** PHRs include ISO date in frontmatter

## Privacy

PHRs may contain:
- Project code and architecture details
- Decision-making rationale
- AI collaboration patterns

Do not include:
- Sensitive credentials or API keys
- Personal identifying information
- Proprietary third-party information

---

**Last Updated:** 2025-12-09
