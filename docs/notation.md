---
title: Notation & Conventions
description: Technical notation, symbols, and conventions used throughout the book
sidebar_position: 997
---

# Notation & Conventions

This document defines the technical notation, symbols, and conventions used throughout the book for consistency and clarity.

## File Paths

### Absolute Paths

Absolute paths from project root are indicated with a leading slash:

```
/docs/introduction.md
/src/components/Layout.tsx
/static/img/logo.svg
```

### Relative Paths

Relative paths are indicated without a leading slash:

```
../components/Button.tsx
./styles.module.css
```

## Code Conventions

### Inline Code

Inline code is enclosed in single backticks:

- File names: `package.json`
- Variable names: `userName`
- Function names: `getData()`
- Commands: `npm install`

### Code Blocks

Code blocks use triple backticks with language specification:

```javascript
// JavaScript example
function greet(name) {
  return `Hello, ${name}!`;
}
```

```typescript
// TypeScript example
interface User {
  name: string;
  age: number;
}
```

### Command Line

Shell commands are prefixed with `$` (don't type the $):

```bash
$ npm install
$ npm run build
```

Output is shown without prefix:

```bash
$ npm run build

> docusaurus build

Build completed successfully!
```

## File Structure Notation

### Directory Trees

Directory structures use tree notation:

```
project/
├── docs/
│   ├── introduction.md
│   └── chapter1/
│       ├── overview.md
│       └── concepts.md
├── src/
│   └── components/
└── package.json
```

### File References

When referencing specific files:

- **Format:** `path/to/file.ext`
- **Example:** `docs/introduction.md`

### Line Numbers

When referencing specific lines in a file:

- **Format:** `file.ext:line_number`
- **Example:** `src/App.tsx:42`
- **Range:** `src/App.tsx:42-58`

## Placeholders

### Template Variables

Template variables use double curly braces:

```
{{VARIABLE_NAME}}
{{DATE_ISO}}
{{FEATURE_NAME}}
```

### User Input

User-provided values are indicated in angle brackets:

```bash
npm install <package-name>
git commit -m "<your-message>"
```

### Optional Parameters

Optional parameters are indicated in square brackets:

```bash
npm run build [--production]
docusaurus deploy [--out-dir <dir>]
```

## Documentation Conventions

### Emphasis

- **Bold:** Important terms, UI elements, file names
  - Example: Click the **Save** button
  
- *Italic:* Emphasis, technical terms on first use
  - Example: This is called *server-side rendering*

### Links

- Internal links: Relative paths to other pages
  - `[Getting Started](../intro.md)`
  
- External links: Full URLs
  - `[React Docs](https://react.dev)`

### Lists

#### Ordered Lists

Use numbered lists for sequential steps:

1. First step
2. Second step
3. Third step

#### Unordered Lists

Use bullet points for non-sequential items:

- Item one
- Item two
- Item three

## Admonitions (Callouts)

### Note

```markdown
:::note
This is a note with general information.
:::
```

### Tip

```markdown
:::tip
This is a helpful tip or best practice.
:::
```

### Warning

```markdown
:::warning
This warns about potential issues.
:::
```

### Danger

```markdown
:::danger
This indicates something critical or dangerous.
:::
```

### Info

```markdown
:::info
This provides additional context or information.
:::
```

## Code Reference Format

### Function Signatures

```
functionName(parameter1: Type, parameter2?: Type): ReturnType
```

Example:

```typescript
getUserById(id: string): Promise<User>
```

### Component Props

```typescript
interface ComponentProps {
  required: string;
  optional?: number;
  callback: (value: string) => void;
}
```

### API Endpoints

```
METHOD /path/to/endpoint
```

Example:

```
GET /api/users/:id
POST /api/users
PUT /api/users/:id
DELETE /api/users/:id
```

## Version Notation

### Semantic Versioning

Version numbers follow semver format:

```
MAJOR.MINOR.PATCH
```

Examples:
- `1.0.0` - Initial release
- `1.1.0` - New feature added (backwards compatible)
- `1.1.1` - Bug fix (backwards compatible)
- `2.0.0` - Breaking change

### Version Ranges

- `^1.2.3` - Compatible with 1.2.3, allows minor and patch updates
- `~1.2.3` - Compatible with 1.2.3, allows only patch updates
- `>=1.2.3` - Greater than or equal to 1.2.3
- `1.2.x` - Any patch version of 1.2

## Status Indicators

### Task Status

- ⬜ Not Started
- 🔵 In Progress
- ✅ Done / Completed
- ❌ Blocked / Failed
- ⚠️ Needs Review

### Document Status

- **Draft** - Work in progress
- **Review** - Awaiting review
- **Approved** - Ready for publication
- **Published** - Live on site
- **Archived** - No longer maintained

## Keyboard Shortcuts

Keyboard shortcuts use `+` to indicate simultaneous keys:

- `Ctrl+C` - Copy
- `Cmd+S` - Save (macOS)
- `Alt+Shift+F` - Format document

## UI Elements

UI elements are indicated in **bold**:

- Click the **Save** button
- Navigate to **Settings** > **Preferences**
- Press **Enter** to submit

## Special Symbols

### Commonly Used

- `→` - Indicates progression or navigation
- `✅` - Correct or completed
- `❌` - Incorrect or failed
- `⚠️` - Warning
- `ℹ️` - Information
- `📝` - Note or documentation
- `🔧` - Configuration or tool
- `🚀` - Deployment or launch

### Mathematical

- `±` - Plus or minus
- `≈` - Approximately equal
- `≠` - Not equal
- `≤` - Less than or equal
- `≥` - Greater than or equal

## Timestamp Format

### ISO 8601

Dates and times use ISO 8601 format:

```
YYYY-MM-DD
2025-12-09

YYYY-MM-DDTHH:mm:ss
2025-12-09T14:30:00
```

## URL Conventions

### Internal Site URLs

```
/path/to/page
/docs/introduction
/blog/2025/12/09/post-title
```

### External URLs

Always include protocol:

```
https://example.com
https://github.com/username/repo
```

## Naming Conventions

### Files

- **Markdown:** `kebab-case.md`
- **TypeScript/JavaScript:** `PascalCase.tsx` (components), `camelCase.ts` (utilities)
- **CSS Modules:** `ComponentName.module.css`
- **Images:** `descriptive-name.svg`

### Variables & Functions

- **Variables:** `camelCase`
- **Constants:** `UPPER_SNAKE_CASE`
- **Functions:** `camelCase`
- **Classes/Components:** `PascalCase`
- **Private fields:** `_privateField`

### Git

- **Branches:** `feature/feature-name`, `fix/bug-description`, `chapter/chapter-name`
- **Commits:** Imperative mood, lowercase: "add feature" not "added feature"

---

**Note:** This notation guide ensures consistency across all content. When introducing new notation, add it to this document first.

**Last Updated:** {{DATE}}
