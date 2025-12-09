---
feature: {{FEATURE_NAME}}
version: {{VERSION}}
status: {{STATUS}}
created: {{DATE_ISO}}
updated: {{DATE_ISO}}
author: {{AUTHOR}}
---

# Implementation Plan: {{FEATURE_NAME}}

## Executive Summary

{{EXECUTIVE_SUMMARY}}

Brief overview of the implementation approach.

## Scope and Dependencies

### In Scope

- {{IN_SCOPE_1}}
- {{IN_SCOPE_2}}
- {{IN_SCOPE_3}}

### Out of Scope

- {{OUT_OF_SCOPE_1}}
- {{OUT_OF_SCOPE_2}}

### Dependencies

#### Internal Dependencies
- {{INTERNAL_DEP_1}}: {{INTERNAL_DEP_1_DESCRIPTION}}
- {{INTERNAL_DEP_2}}: {{INTERNAL_DEP_2_DESCRIPTION}}

#### External Dependencies
- {{EXTERNAL_DEP_1}}: {{EXTERNAL_DEP_1_DESCRIPTION}}
- {{EXTERNAL_DEP_2}}: {{EXTERNAL_DEP_2_DESCRIPTION}}

## Architecture & Design Decisions

### Key Decisions

#### Decision 1: {{DECISION_1_NAME}}

**Options Considered:**
1. {{DECISION_1_OPTION_1}}
2. {{DECISION_1_OPTION_2}}
3. {{DECISION_1_OPTION_3}}

**Trade-offs:**
- {{DECISION_1_TRADEOFF_1}}
- {{DECISION_1_TRADEOFF_2}}

**Rationale:**
{{DECISION_1_RATIONALE}}

**ADR:** {{DECISION_1_ADR_LINK}}

#### Decision 2: {{DECISION_2_NAME}}

**Options Considered:**
1. {{DECISION_2_OPTION_1}}
2. {{DECISION_2_OPTION_2}}

**Trade-offs:**
- {{DECISION_2_TRADEOFF_1}}

**Rationale:**
{{DECISION_2_RATIONALE}}

**ADR:** {{DECISION_2_ADR_LINK}}

## Content Structure

### Chapter/Section Organization

```
{{FEATURE_NAME}}/
├── index.md (Overview)
├── {{SECTION_1}}.md
├── {{SECTION_2}}.md
├── {{SECTION_3}}.md
└── summary.md
```

### Content Flow

1. **Introduction** ({{INTRO_WORD_COUNT}} words)
   - {{INTRO_TOPIC_1}}
   - {{INTRO_TOPIC_2}}

2. **Core Concepts** ({{CORE_WORD_COUNT}} words)
   - {{CORE_TOPIC_1}}
   - {{CORE_TOPIC_2}}
   - {{CORE_TOPIC_3}}

3. **Practical Examples** ({{EXAMPLES_WORD_COUNT}} words)
   - {{EXAMPLE_TOPIC_1}}
   - {{EXAMPLE_TOPIC_2}}

4. **Hands-on Tutorial** ({{TUTORIAL_WORD_COUNT}} words)
   - {{TUTORIAL_TOPIC_1}}
   - {{TUTORIAL_TOPIC_2}}

5. **Summary & Exercises** ({{SUMMARY_WORD_COUNT}} words)
   - Key takeaways
   - Practice exercises

## Technical Implementation

### Code Examples

#### Example 1: {{CODE_EXAMPLE_1_NAME}}
- **File:** `examples/{{FEATURE_NAME}}/{{CODE_EXAMPLE_1_FILE}}`
- **Language:** {{CODE_EXAMPLE_1_LANGUAGE}}
- **Purpose:** {{CODE_EXAMPLE_1_PURPOSE}}
- **Dependencies:** {{CODE_EXAMPLE_1_DEPS}}
- **Test Strategy:** {{CODE_EXAMPLE_1_TESTS}}

#### Example 2: {{CODE_EXAMPLE_2_NAME}}
- **File:** `examples/{{FEATURE_NAME}}/{{CODE_EXAMPLE_2_FILE}}`
- **Language:** {{CODE_EXAMPLE_2_LANGUAGE}}
- **Purpose:** {{CODE_EXAMPLE_2_PURPOSE}}
- **Dependencies:** {{CODE_EXAMPLE_2_DEPS}}
- **Test Strategy:** {{CODE_EXAMPLE_2_TESTS}}

### Visual Assets

#### Diagram 1: {{DIAGRAM_1_NAME}}
- **File:** `static/img/{{FEATURE_NAME}}/{{DIAGRAM_1_FILE}}`
- **Type:** {{DIAGRAM_1_TYPE}}
- **Tool:** {{DIAGRAM_1_TOOL}} (e.g., draw.io, Figma, Mermaid)
- **Format:** SVG (optimized)
- **Alt Text:** {{DIAGRAM_1_ALT_TEXT}}

#### Diagram 2: {{DIAGRAM_2_NAME}}
- **File:** `static/img/{{FEATURE_NAME}}/{{DIAGRAM_2_FILE}}`
- **Type:** {{DIAGRAM_2_TYPE}}
- **Tool:** {{DIAGRAM_2_TOOL}}
- **Format:** SVG (optimized)
- **Alt Text:** {{DIAGRAM_2_ALT_TEXT}}

## UI/UX Considerations

### Custom Components Needed

1. **{{COMPONENT_1_NAME}}**
   - **File:** `src/components/{{COMPONENT_1_FILE}}.tsx`
   - **Purpose:** {{COMPONENT_1_PURPOSE}}
   - **Props:** {{COMPONENT_1_PROPS}}

2. **{{COMPONENT_2_NAME}}**
   - **File:** `src/components/{{COMPONENT_2_FILE}}.tsx`
   - **Purpose:** {{COMPONENT_2_PURPOSE}}
   - **Props:** {{COMPONENT_2_PROPS}}

### Theme Customization

- Custom CSS: `src/css/{{FEATURE_NAME}}.css`
- Colors: {{CUSTOM_COLORS}}
- Typography: {{CUSTOM_TYPOGRAPHY}}

## Quality & Testing

### Testing Strategy

- [ ] Code examples tested locally
- [ ] Links validated
- [ ] Images optimized (< 200KB)
- [ ] Mobile responsive verified
- [ ] Dark mode compatibility checked
- [ ] Accessibility audit (WCAG AA)
- [ ] Spell check passed
- [ ] Technical accuracy review

### Review Checkpoints

1. **Technical Review** ({{TECHNICAL_REVIEWER}})
   - Code accuracy
   - Technical claims validation
   
2. **Peer Review** ({{PEER_REVIEWER}})
   - Clarity and flow
   - Learning objectives met

3. **Editorial Review** ({{EDITOR}})
   - Consistency with style guide
   - Grammar and formatting

## Risk Analysis

### Top Risks

1. **{{RISK_1_NAME}}**
   - **Impact:** High / Medium / Low
   - **Probability:** High / Medium / Low
   - **Mitigation:** {{RISK_1_MITIGATION}}

2. **{{RISK_2_NAME}}**
   - **Impact:** High / Medium / Low
   - **Probability:** High / Medium / Low
   - **Mitigation:** {{RISK_2_MITIGATION}}

3. **{{RISK_3_NAME}}**
   - **Impact:** High / Medium / Low
   - **Probability:** High / Medium / Low
   - **Mitigation:** {{RISK_3_MITIGATION}}

## Complexity Justifications

> Document any deviations from constitution principles here

### Deviation 1: {{DEVIATION_1_NAME}}

- **Principle Violated:** {{DEVIATION_1_PRINCIPLE}}
- **Why Necessary:** {{DEVIATION_1_REASON}}
- **Alternative Rejected:** {{DEVIATION_1_ALTERNATIVE}}
- **Migration Path:** {{DEVIATION_1_MIGRATION}}

## Timeline & Milestones

**Note:** Milestones define completion criteria, not time estimates.

### Phase 1: Content Creation
- [ ] Draft introduction
- [ ] Draft core concepts sections
- [ ] Create code examples
- [ ] Create diagrams

### Phase 2: Review & Refinement
- [ ] Technical review complete
- [ ] Peer review complete
- [ ] Revisions implemented

### Phase 3: Production
- [ ] Quality gates passed
- [ ] Metadata complete
- [ ] Build validation passed

## Success Criteria

This implementation is complete when:

- [ ] All spec requirements met
- [ ] Learning objectives addressed
- [ ] Code examples tested and functional
- [ ] All diagrams created and optimized
- [ ] Quality gates passed
- [ ] Reviews completed
- [ ] Documentation complete
- [ ] Deployed to staging
- [ ] Ready for production

## Related Documentation

- **Spec:** `specs/{{FEATURE_NAME}}/spec.md`
- **Tasks:** `specs/{{FEATURE_NAME}}/tasks.md`
- **ADRs:** {{ADR_LINKS}}
- **PHRs:** `history/prompts/{{FEATURE_NAME}}/`

---

**Status:** {{STATUS}}
**Version:** {{VERSION}}
**Last Updated:** {{DATE_ISO}}
