# Code Examples

This directory contains all code examples used throughout the book.

## Directory Structure

Examples are organized by chapter/feature:

```
examples/
├── <chapter-name>/
│   ├── <example-name>/
│   │   ├── README.md          # Example documentation
│   │   ├── package.json       # Dependencies (if applicable)
│   │   ├── src/               # Source code
│   │   └── tests/             # Tests (if applicable)
│   └── solutions/             # Exercise solutions
└── README.md
```

## Example Organization

Each example should:
- Be self-contained and runnable
- Include a README explaining purpose and usage
- List all dependencies with versions
- Include comments explaining WHY not WHAT
- Handle errors appropriately
- Follow security best practices (no hardcoded secrets)

## Example Types

### 1. Inline Examples

Small snippets embedded directly in chapter content:

```javascript
// Simple example
function greet(name) {
  return `Hello, ${name}!`;
}
```

### 2. Standalone Examples

Complete projects in their own directories:

```
examples/docusaurus-basics/hello-world/
├── README.md
├── package.json
├── docusaurus.config.js
└── docs/
    └── intro.md
```

### 3. Interactive Examples

Examples that can be run in browser using:
- CodeSandbox
- StackBlitz
- Docusaurus Playground

### 4. Exercise Solutions

Sample solutions for chapter exercises:

```
examples/<chapter>/solutions/
├── exercise-1-solution/
├── exercise-2-solution/
└── exercise-3-solution/
```

## Creating a New Example

### Step 1: Create Directory

```bash
mkdir -p examples/<chapter-name>/<example-name>
```

### Step 2: Add README

```markdown
# Example: <Example Name>

## Purpose

What this example demonstrates.

## Prerequisites

- Node.js 18+
- npm 9+
- (other requirements)

## Installation

\```bash
npm install
\```

## Usage

\```bash
npm start
\```

## Key Concepts

- Concept 1: Explanation
- Concept 2: Explanation

## Related Chapter

See [Chapter Name](../../docs/<chapter>/page.md)
```

### Step 3: Add Code

Write clean, well-documented code following constitution principles.

### Step 4: Test

Ensure the example runs successfully and produces expected output.

### Step 5: Document

Add inline comments and update README with:
- Purpose and learning objectives
- Installation steps
- Usage instructions
- Key concepts demonstrated
- Link to related chapter

## Code Quality Standards

### All Examples Must:

- ✅ Be tested and functional
- ✅ Include README with usage instructions
- ✅ List dependencies with versions
- ✅ Use meaningful variable names
- ✅ Include error handling
- ✅ Follow language-specific style guides
- ✅ Have no hardcoded secrets/credentials
- ✅ Include comments explaining WHY

### Examples Should:

- Be as simple as possible while demonstrating the concept
- Focus on one concept at a time
- Use standard libraries when possible
- Include inline comments for complex logic
- Provide expected output or behavior
- Be cross-platform compatible

## Language-Specific Guidelines

### JavaScript/TypeScript

- Use ES6+ syntax
- Prefer `const` over `let`
- Use TypeScript for complex examples
- Follow Airbnb or Standard style guide
- Include `package.json` with exact versions

### Python

- Follow PEP 8 style guide
- Use type hints for function signatures
- Include `requirements.txt` with pinned versions
- Use virtual environments

### Shell/Bash

- Make scripts executable (`chmod +x`)
- Include shebang line (`#!/bin/bash`)
- Check for required commands
- Provide usage instructions

## Dependencies

### Managing Dependencies

```json
{
  "dependencies": {
    "react": "18.2.0",
    "docusaurus": "3.0.0"
  }
}
```

- Pin exact versions for reproducibility
- Document why each dependency is needed
- Keep dependencies minimal
- Regular security updates

### Version Specifications

```
# Requires: Node.js >= 18.0.0, npm >= 9.0.0
# Dependencies: docusaurus@3.0.0, react@18.2.0
```

## Testing Examples

### Manual Testing

Before committing, verify:

1. Fresh install works: `rm -rf node_modules && npm install`
2. Code runs without errors
3. Output matches documentation
4. Works on different platforms (if applicable)

### Automated Testing

Include tests where appropriate:

```
examples/<chapter>/<example>/
├── src/
│   └── example.js
└── tests/
    └── example.test.js
```

## README Template

```markdown
# Example: <Name>

## Purpose
Brief description of what this example demonstrates.

## Prerequisites
- Requirement 1
- Requirement 2

## Installation
\```bash
npm install
# or other install command
\```

## Usage
\```bash
npm start
# or other run command
\```

## Expected Output
\```
Output here
\```

## Key Concepts
- Concept 1: Explanation
- Concept 2: Explanation

## Code Structure
\```
src/
├── index.js       # Main entry point
├── component.js   # Example component
└── utils.js       # Utility functions
\```

## Related Documentation
- [Chapter Name](../../docs/<chapter>/<page>.md)
- [External Resource](https://example.com)

## Troubleshooting

**Issue:** Error message
**Solution:** How to fix

## License
Same as main project
```

## Best Practices

1. **Start Simple:** Begin with the simplest version that demonstrates the concept
2. **Build Up:** Add complexity incrementally in separate examples
3. **Self-Contained:** Each example should run independently
4. **Document Why:** Explain why this approach, not just what it does
5. **Real-World:** Use realistic scenarios when possible
6. **Accessible:** Ensure examples work for target audience skill level
7. **Maintained:** Keep examples updated with dependency versions

## Interactive Examples

For browser-based examples, include links:

```markdown
Try this example:
- [CodeSandbox](https://codesandbox.io/s/...)
- [StackBlitz](https://stackblitz.com/edit/...)
```

## Security

### Never Include:

- API keys or secrets
- Passwords or credentials
- Personal identifying information
- Sensitive configuration

### Use Instead:

- Environment variables (`.env.example`)
- Placeholder values with clear documentation
- Mock data for demonstrations

## Linking Examples in Content

Reference examples from chapter content:

```markdown
See the complete example in [examples/chapter-name/example-name](../examples/chapter-name/example-name).
```

---

**Last Updated:** 2025-12-09
