---
id: {{ID}}
title: "{{TITLE}}"
stage: {{STAGE}}
date: {{DATE_ISO}}
surface: {{SURFACE}}
model: {{MODEL}}
feature: {{FEATURE}}
branch: {{BRANCH}}
user: {{USER}}
command: {{COMMAND}}
labels: {{LABELS}}
links:
  spec: {{SPEC_LINK}}
  ticket: {{TICKET_LINK}}
  adr: {{ADR_LINK}}
  pr: {{PR_LINK}}
files:
{{FILES_YAML}}
tests:
{{TESTS_YAML}}
---

# {{TITLE}}

## Context

**Stage:** {{STAGE}}
**Feature:** {{FEATURE}}
**Date:** {{DATE_ISO}}
**User:** {{USER}}
**Model:** {{MODEL}}
**Command:** {{COMMAND}}

## Prompt

```
{{PROMPT_TEXT}}
```

## Response Summary

{{RESPONSE_TEXT}}

## Files Modified/Created

{{FILES_YAML}}

## Tests Run/Added

{{TESTS_YAML}}

## Outcome

{{OUTCOME}}

## Links

- **Spec:** {{SPEC_LINK}}
- **Ticket:** {{TICKET_LINK}}
- **ADR:** {{ADR_LINK}}
- **PR:** {{PR_LINK}}

## Labels

{{LABELS}}

---

*Generated with SpecKit Plus | PHR ID: {{ID}}*
