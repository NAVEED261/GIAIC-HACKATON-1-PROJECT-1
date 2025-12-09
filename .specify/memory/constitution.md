# Physical AI Textbook Constitution

**Project:** AI/Spec-Driven Book Creation using Docusaurus and GitHub Pages
**Version:** 1.0.0
**Ratified:** 2025-12-09
**Last Amended:** 2025-12-09

## Core Principles

### I. Content Accuracy & Technical Rigor

**Mandate:** All technical content must be verified, tested, and reflect current best practices.

- Code examples must be complete, executable, and tested before publication
- Hardware specifications must match official vendor documentation
- Software version numbers must be explicitly stated (e.g., "ROS 2 Humble", "Ubuntu 22.04 LTS")
- External links must be validated quarterly and updated if broken
- Mathematical notation must follow standard conventions (define custom notation in appendix)
- Claims about performance, capabilities, or compatibility require citation or testing evidence

**Rationale:** Students rely on textbook accuracy for learning and implementation. Errors erode trust and waste student time.

---

### II. Educational Clarity & Accessibility

**Mandate:** Content must be understandable to the target audience (industry practitioners with Python knowledge).

- Use consistent terminology throughout (maintain glossary for technical terms)
- Provide context before introducing complex concepts ("Why this matters" sections)
- Include visual aids (diagrams, flowcharts, architecture drawings) for system-level concepts
- Offer multiple explanations for difficult concepts (intuitive explanation + formal definition + analogy)
- Learning objectives must be measurable and stated upfront for each chapter
- Avoid jargon; when unavoidable, define terms on first use and add to glossary

**Rationale:** Accessibility ensures practitioners from diverse backgrounds can learn effectively, maximizing course impact.

---

### III. Consistency & Standards

**Mandate:** Maintain uniform structure, style, and formatting across all chapters.

- Chapter structure template: Introduction → Prerequisites → Learning Objectives → Content → Exercises → Summary → References
- Code formatting: Follow PEP 8 (Python), Google Style Guide (C++), Black formatter for consistency
- Headings: Use sentence case for all headings (e.g., "Introduction to ROS 2", not "Introduction To ROS 2")
- Frontmatter metadata: All chapters must include `week`, `module`, `estimated_time`, `prerequisites`, `learning_objectives`
- File naming: Use kebab-case for all markdown files (e.g., `ros2-fundamentals.md`, not `ROS2_Fundamentals.md`)
- Image naming: `{chapter-slug}-{description}.{svg|png}` (e.g., `ros2-fundamentals-node-architecture.svg`)

**Rationale:** Consistency reduces cognitive load, making it easier for students to navigate and instructors to customize.

---

### IV. Docusaurus Structure & Quality

**Mandate:** Leverage Docusaurus features for optimal documentation experience.

- Sidebar navigation must be intuitive and support access by week, module, and topic
- Use MDX for interactive components (glossary search, module cards, prerequisite graphs)
- Optimize images: SVG preferred for diagrams, PNG < 200KB for screenshots
- Dark mode compatibility required for all custom components
- Lighthouse scores: Performance ≥ 90, Accessibility ≥ 90, SEO ≥ 90
- Build must complete without warnings or errors
- Link checking enforced in CI/CD (no broken internal links)
- Mobile responsiveness required (test on 375px, 768px, 1024px, 1440px viewports)

**Rationale:** Docusaurus provides best-in-class documentation features; proper usage ensures professional presentation and discoverability.

---

### V. Code Example Quality

**Mandate:** Code examples must be production-ready, well-documented, and pedagogically sound.

- Every code block includes language specification (```python, ```bash, ```yaml)
- Dependencies listed with version specifications (`rospy==1.15.0`, `numpy>=1.21`)
- Comments explain WHY, not WHAT (assume reader can read code)
- Error handling included for realistic scenarios (network failures, sensor noise, etc.)
- Variable names are descriptive (avoid `x`, `temp`, `data`; use `joint_angle`, `sensor_reading`)
- Complete examples: Include imports, setup, main logic, and cleanup
- Testing: All code examples tested in target environment before publication

**Embedded Code Guidelines:**
- Fenced code blocks with syntax highlighting
- Comments indicate dependencies and setup requirements
- Runnable as-is or with minimal setup (documented in surrounding text)

**Rationale:** Students learn by running and modifying examples. Incomplete or broken code frustrates learners and damages credibility.

---

### VI. UI/UX Excellence

**Mandate:** User experience must prioritize ease of navigation, clarity, and engagement.

- Homepage dashboard with module cards showing week range, learning outcomes, and estimated time
- Quick links sidebar always accessible (hardware setup, assessments, glossary, troubleshooting)
- Glossary search component provides instant term lookup (< 2 seconds response time)
- Prerequisite chains visible in sidebar and chapter frontmatter
- Table of contents auto-generated for chapters > 500 words
- Breadcrumbs enabled for all pages
- Search functionality: Hybrid approach (Algolia DocSearch for content + custom glossary search)
- No page should require > 2 clicks from homepage (measured and enforced)

**Rationale:** Friction in navigation reduces engagement. Intuitive UX keeps students focused on learning, not fighting the interface.

---

### VII. Deployment & Publishing Standards

**Mandate:** Deployment must be automated, reliable, and maintain quality gates.

- GitHub Actions CI/CD pipeline required for all deployments
- Pre-deployment checks: Build success, link validation, spell check, Lighthouse audit
- Staging environment for preview before production deployment
- Incremental content delivery supported (Week 1-2 can be published independently)
- Draft mode for unreleased content (`draft: true` in frontmatter)
- GitHub Pages deployment with custom domain support (optional)
- Rollback capability: Keep last 3 production builds
- Performance budget enforced: LCP < 2.5s, CLS < 0.1, FID < 100ms

**CI/CD Quality Gates:**
1. Build completes without errors/warnings
2. All internal links resolve (no 404s)
3. Spell check passes (robotics dictionary configured)
4. Lighthouse scores meet thresholds (≥ 90 for performance, accessibility, SEO)
5. Accessibility audit passes (WCAG AA compliance)

**Rationale:** Automated deployment ensures consistency, reduces human error, and maintains quality standards across all releases.

---

### VIII. AI-Driven Content Standards

**Mandate:** When using AI tools for content generation or assistance, maintain human oversight and quality control.

- AI-generated content must be reviewed, tested, and validated by human subject matter expert
- Code generated by AI must be tested in target environment before inclusion
- Factual claims from AI must be verified against authoritative sources
- AI-assisted diagrams/visuals require review for accuracy and clarity
- Attribution: Note when AI tools were used for significant content generation (not required for minor edits)
- Bias check: Review AI-generated content for potential bias or assumptions
- Clarity review: AI explanations often lack context; ensure human reviewer adds necessary framing

**Rationale:** AI accelerates content creation but lacks domain expertise and can introduce errors. Human oversight ensures accuracy and pedagogical effectiveness.

---

## Governance

### Amendment Process

1. Propose amendment with rationale and impact assessment
2. Review by curriculum designer and technical lead
3. Approval requires consensus (or designated authority if time-critical)
4. Document in ADR (Architecture Decision Record)
5. Update constitution version (MAJOR for breaking changes, MINOR for additions)
6. Communicate changes to all contributors

### Enforcement

- All pull requests must pass automated quality gates (CI/CD checks)
- Code reviews verify compliance with constitution principles
- Non-compliance blocks merge until resolved
- Exceptions require explicit justification documented in PR description

### Deviation Policy

- Deviations from constitution require documented justification
- Temporary deviations allowed for prototyping; must be resolved before production
- Permanent deviations require amendment process (see above)

---

**Version:** 1.0.0
**Ratified:** 2025-12-09
**Last Amended:** 2025-12-09
**Next Review:** 2026-03-09 (Quarterly review cycle)
