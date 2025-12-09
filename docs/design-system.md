---
title: Design System
description: Visual design standards, components, and UI guidelines for the book
sidebar_position: 998
---

# Design System

This document defines the visual design standards, components, and UI guidelines used throughout the book's website.

## Brand Identity

### Colors

#### Primary Colors

- **Primary:** `#3578e5` (Docusaurus Blue)
  - Used for: Primary CTAs, links, active states
  - Contrast ratio: 4.5:1 (WCAG AA compliant)

- **Secondary:** `#25c2a0` (Teal)
  - Used for: Accent elements, success states
  
#### Neutral Colors

- **Text (Light Mode):** `#1c1e21` (Near Black)
- **Text (Dark Mode):** `#e3e3e3` (Off White)
- **Background (Light Mode):** `#ffffff` (White)
- **Background (Dark Mode):** `#1b1b1d` (Dark Gray)
- **Border:** `#e3e3e3` (Light Gray)

#### Semantic Colors

- **Success:** `#00c853` (Green)
- **Warning:** `#ffb300` (Amber)
- **Error:** `#d32f2f` (Red)
- **Info:** `#2196f3` (Blue)

### Typography

#### Font Families

- **Headings:** `'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif`
- **Body:** `'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif`
- **Code:** `'Fira Code', 'Courier New', monospace`

#### Font Sizes

- **H1:** 2.5rem (40px)
- **H2:** 2rem (32px)
- **H3:** 1.5rem (24px)
- **H4:** 1.25rem (20px)
- **H5:** 1.125rem (18px)
- **Body:** 1rem (16px)
- **Small:** 0.875rem (14px)

#### Font Weights

- **Regular:** 400
- **Medium:** 500
- **Semibold:** 600
- **Bold:** 700

#### Line Heights

- **Headings:** 1.2
- **Body:** 1.6
- **Code:** 1.4

### Spacing

Using an 8px base unit:

- **xs:** 4px (0.25rem)
- **sm:** 8px (0.5rem)
- **md:** 16px (1rem)
- **lg:** 24px (1.5rem)
- **xl:** 32px (2rem)
- **2xl:** 48px (3rem)
- **3xl:** 64px (4rem)

## Components

### Buttons

#### Primary Button

```css
.button-primary {
  background-color: #3578e5;
  color: #ffffff;
  padding: 12px 24px;
  border-radius: 8px;
  font-weight: 600;
  transition: background-color 0.2s;
}

.button-primary:hover {
  background-color: #2563eb;
}
```

#### Secondary Button

```css
.button-secondary {
  background-color: transparent;
  color: #3578e5;
  border: 2px solid #3578e5;
  padding: 12px 24px;
  border-radius: 8px;
  font-weight: 600;
  transition: all 0.2s;
}

.button-secondary:hover {
  background-color: #3578e5;
  color: #ffffff;
}
```

### Cards

```css
.card {
  background-color: var(--ifm-card-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 12px;
  padding: 24px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05);
  transition: box-shadow 0.2s;
}

.card:hover {
  box-shadow: 0 4px 16px rgba(0, 0, 0, 0.1);
}
```

### Code Blocks

```css
.code-block {
  background-color: var(--ifm-code-background);
  border-radius: 8px;
  padding: 16px;
  overflow-x: auto;
  font-family: 'Fira Code', monospace;
  font-size: 14px;
  line-height: 1.4;
}
```

### Admonitions (Callouts)

- **Note:** Blue background, info icon
- **Tip:** Green background, lightbulb icon
- **Warning:** Yellow background, warning icon
- **Danger:** Red background, alert icon

## Layout

### Grid System

- **Container max-width:** 1320px
- **Gutter:** 24px
- **Columns:** 12-column grid

### Breakpoints

- **Mobile:** < 768px
- **Tablet:** 768px - 1023px
- **Desktop:** 1024px - 1439px
- **Wide:** ≥ 1440px

### Sidebar

- **Width:** 300px (Desktop)
- **Collapsed:** 0px (Mobile)
- **Transition:** 0.3s ease

## Accessibility

### Contrast Ratios

All color combinations must meet WCAG AA standards:
- **Normal text:** Minimum 4.5:1
- **Large text (18px+):** Minimum 3:1
- **UI components:** Minimum 3:1

### Focus States

All interactive elements must have visible focus indicators:

```css
:focus {
  outline: 2px solid #3578e5;
  outline-offset: 2px;
}
```

### Motion

Respect user preferences for reduced motion:

```css
@media (prefers-reduced-motion: reduce) {
  * {
    animation-duration: 0.01ms !important;
    transition-duration: 0.01ms !important;
  }
}
```

## Icons

### Icon Library

- **Library:** Feather Icons or Heroicons
- **Size:** 20px (default), 16px (small), 24px (large)
- **Stroke width:** 2px

### Usage Guidelines

- Use icons to support text, not replace it
- Ensure icons have appropriate aria-labels
- Maintain consistent icon style throughout

## Images

### Formats

- **Photos:** WebP (with JPG fallback)
- **Diagrams:** SVG
- **Icons:** SVG

### Optimization

- **Maximum file size:** 200KB
- **Responsive images:** Provide multiple sizes
- **Alt text:** Always required and descriptive

### Aspect Ratios

- **Hero images:** 16:9
- **Thumbnails:** 1:1
- **Diagrams:** Vary based on content

## Animation

### Timing Functions

- **Ease-in-out:** Default for most animations
- **Ease-out:** For entering elements
- **Ease-in:** For exiting elements

### Durations

- **Fast:** 150ms (Hover effects)
- **Normal:** 250ms (Transitions)
- **Slow:** 400ms (Complex animations)

### Principles

- Keep animations subtle and purposeful
- Avoid animations that distract from content
- Ensure animations enhance UX, not hinder it

## Dark Mode

All components must support both light and dark themes:

```css
/* Use CSS variables for theme-aware colors */
:root {
  --text-color: #1c1e21;
  --background-color: #ffffff;
}

[data-theme='dark'] {
  --text-color: #e3e3e3;
  --background-color: #1b1b1d;
}
```

## Custom Components

### Location

All custom React components live in `/src/components/`

### Naming Convention

- **Files:** PascalCase (e.g., `BookLayout.tsx`)
- **Components:** PascalCase (e.g., `<BookLayout />`)

### Structure

```tsx
// src/components/ExampleComponent.tsx
import React from 'react';
import styles from './ExampleComponent.module.css';

interface ExampleComponentProps {
  title: string;
  description?: string;
}

export default function ExampleComponent({ 
  title, 
  description 
}: ExampleComponentProps): JSX.Element {
  return (
    <div className={styles.container}>
      <h2>{title}</h2>
      {description && <p>{description}</p>}
    </div>
  );
}
```

---

**Note:** This design system is a living document and should be updated as new patterns and components are introduced.

**Last Updated:** {{DATE}}
