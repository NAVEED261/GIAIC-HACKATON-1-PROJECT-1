# Architectural Decision Records (ADRs)

This directory contains Architectural Decision Records (ADRs) that document significant architectural and design decisions made during the development of this book.

## Purpose

ADRs capture:
- **Context:** What situation led to the decision
- **Decision:** What was decided
- **Alternatives:** What other options were considered
- **Rationale:** Why this decision was chosen
- **Consequences:** What are the impacts (positive and negative)

## When to Create an ADR

Create an ADR when ALL three conditions are met:

1. **Impact:** The decision has long-term consequences
   - Examples: Framework choice, deployment platform, content organization strategy
   
2. **Alternatives:** Multiple viable options with significant tradeoffs
   - Examples: Docusaurus vs. GitBook, TypeScript vs. JavaScript
   
3. **Scope:** The decision affects multiple chapters or overall book structure
   - Examples: Chapter organization, design system approach, versioning strategy

## ADR Template

ADRs follow the template defined in `.specify/templates/adr-template.md`.

Required sections:
- Status (Proposed, Accepted, Rejected, Superseded, Deprecated)
- Context and problem statement
- Decision
- Alternatives considered (with pros/cons)
- Rationale
- Consequences (positive, negative, neutral)
- Implementation notes
- Related decisions

## ADR Naming Convention

```
ADR-<ID>-<title-in-kebab-case>.md
```

Examples:
- `ADR-001-choose-docusaurus-framework.md`
- `ADR-002-typescript-for-all-components.md`
- `ADR-003-chapter-organization-by-complexity.md`

## ADR Status

- **Proposed:** Under discussion
- **Accepted:** Approved and being implemented
- **Rejected:** Decided against
- **Superseded:** Replaced by a newer ADR
- **Deprecated:** No longer applicable

## ADR Lifecycle

1. **Proposal:** ADR created with status "Proposed"
2. **Discussion:** Team reviews and discusses
3. **Decision:** ADR status updated (Accepted/Rejected)
4. **Implementation:** Accepted ADRs guide development
5. **Review:** ADRs reviewed periodically for relevance

## Example ADRs for This Project

Potential ADRs might include:

- **Documentation Framework Choice**
  - Docusaurus vs. GitBook vs. VuePress vs. Nextra
  
- **Deployment Platform**
  - GitHub Pages vs. Netlify vs. Vercel
  
- **Content Organization**
  - By topic vs. by difficulty level vs. hybrid
  
- **Code Language**
  - JavaScript vs. TypeScript for examples
  
- **Design System**
  - Custom theme vs. default Docusaurus theme
  
- **AI Tool Selection**
  - Which AI tools to use for content creation
  
- **Version Control**
  - Monorepo vs. separate repos for content and code

## Linking ADRs

ADRs can reference each other:
- **Supersedes:** This ADR replaces ADR-XXX
- **Superseded by:** This ADR was replaced by ADR-XXX
- **Related to:** This ADR is related to ADR-XXX
- **Amends:** This ADR modifies ADR-XXX

## Best Practices

1. **Be specific:** Focus on one decision per ADR
2. **Show tradeoffs:** Clearly explain pros/cons of each option
3. **Be honest:** Include negative consequences
4. **Update status:** Keep ADR status current
5. **Link liberally:** Reference related specs, tasks, PRs
6. **Date everything:** Include decision date
7. **Review regularly:** Ensure ADRs remain relevant

## Template Usage

To create a new ADR:

```bash
# Option 1: Use the command
/sp.adr <decision-title>

# Option 2: Copy the template manually
cp .specify/templates/adr-template.md history/adr/ADR-XXX-title.md
```

## Reading Order

ADRs are numbered sequentially. To understand the project's architectural evolution, read them in order:

1. ADR-001 (First decision)
2. ADR-002 (Second decision)
3. ...

## Searching ADRs

Find ADRs by:
- **Number:** ADR-XXX
- **Status:** Search for "Status: Accepted"
- **Topic:** Search content for keywords
- **Date:** ADRs include ISO date in frontmatter

---

**Last Updated:** 2025-12-09
