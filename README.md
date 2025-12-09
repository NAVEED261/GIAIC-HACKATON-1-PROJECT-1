# Physical AI Textbook

> **A comprehensive 13-week course textbook for learning Physical AI, Humanoid Robotics, and Vision-Language-Action models**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Docusaurus](https://img.shields.io/badge/Built%20with-Docusaurus-3ECC5F.svg)](https://docusaurus.io/)
[![GitHub Pages](https://img.shields.io/badge/Deployed%20on-GitHub%20Pages-blue.svg)](https://naveed261.github.io/GIAIC-HACKATON-1-PROJECT-1/)

**Live Website:** [https://naveed261.github.io/GIAIC-HACKATON-1-PROJECT-1/](https://naveed261.github.io/GIAIC-HACKATON-1-PROJECT-1/)

---

## 📖 Overview

The **Physical AI Textbook** is an open-source, interactive educational platform designed to teach industry practitioners and students how to build autonomous humanoid robotic systems. This comprehensive course covers everything from ROS 2 fundamentals to advanced Vision-Language-Action (VLA) integration.

### Key Features

- 🤖 **4 Core Modules** covering ROS 2, Digital Twin Simulation, NVIDIA Isaac Sim, and VLA Models
- 📚 **13-Week Curriculum** with 130-156 hours of hands-on learning
- 🎯 **Capstone Project** - Build a complete autonomous humanoid system
- 💻 **3 Hardware Options** - Workstation, Jetson Orin Nano, or Cloud GPU
- 🔍 **Interactive Components** - Module cards, quick links panel, and responsive navigation
- 🌓 **Dark/Light Mode** support for comfortable reading
- 📱 **Mobile Responsive** design for learning on any device

---

## 🎓 Course Structure

### Module 1: ROS 2 Fundamentals (Weeks 3-5)
- Master nodes, topics, services, and actions
- Build multi-node robotic systems
- Learn ROS 2 communication patterns

### Module 2: Digital Twin Simulation (Weeks 6-7)
- Gazebo and Unity ML-Agents
- Sim-to-real transfer techniques
- Safe testing before hardware deployment

### Module 3: NVIDIA Isaac Sim (Weeks 8-10)
- High-fidelity perception pipelines
- Object detection and pose estimation
- GPU-accelerated physics simulation

### Module 4: Vision-Language-Action Models (Weeks 11-13)
- Natural language robot control
- Task decomposition from language commands
- **Capstone:** Autonomous humanoid integration

---

## 🛠️ Tech Stack

| Technology | Purpose |
|------------|---------|
| **Docusaurus v3** | Static site generator with React |
| **TypeScript** | Type-safe component development |
| **React 18** | Custom interactive components |
| **Node.js 18+** | Build tooling and development server |
| **GitHub Pages** | Free hosting and deployment |
| **Markdown** | Content authoring |
| **CSS Variables** | Module-specific theming |

---

## 🚀 Getting Started

### Prerequisites

- **Node.js** 18.0 or higher
- **npm** or **yarn** package manager
- **Git** for version control

### Installation

```bash
# Clone the repository
git clone https://github.com/NAVEED261/GIAIC-HACKATON-1-PROJECT-1.git

# Navigate to project directory
cd GIAIC-HACKATON-1-PROJECT-1/physical-ai-textbook

# Install dependencies
npm install

# Start development server
npm start
```

The website will open at `http://localhost:3000/GIAIC-HACKATON-1-PROJECT-1/`

### Build for Production

```bash
# Create optimized production build
npm run build

# Test production build locally
npm run serve
```

---

## 📂 Project Structure

```
GIAIC-HACKATON-1-PROJECT-1/
├── physical-ai-textbook/           # Main Docusaurus project
│   ├── docs/                       # Course content (Markdown)
│   │   ├── intro/                  # Introduction (Weeks 1-2)
│   │   ├── setup/                  # Hardware & software setup guides
│   │   ├── module-1-ros2/          # ROS 2 content (Weeks 3-5)
│   │   ├── module-2-digital-twin/  # Simulation (Weeks 6-7)
│   │   ├── module-3-isaac/         # Isaac Sim (Weeks 8-10)
│   │   ├── module-4-vla/           # VLA models (Weeks 11-13)
│   │   ├── capstone/               # Capstone project
│   │   ├── assessments/            # Grading rubrics
│   │   └── reference/              # Glossary & troubleshooting
│   ├── src/
│   │   ├── components/             # Custom React components
│   │   │   ├── ModuleCard.tsx      # Module display cards
│   │   │   └── QuickLinksPanel.tsx # Sidebar navigation
│   │   ├── pages/                  # Custom pages
│   │   │   └── index.tsx           # Homepage dashboard
│   │   └── css/                    # Styling
│   ├── static/
│   │   └── data/
│   │       └── modules.json        # Module metadata
│   ├── docusaurus.config.ts        # Site configuration
│   └── sidebars.ts                 # Sidebar structure
├── specs/                          # Feature specifications
│   └── 1-physical-ai-textbook-structure/
│       ├── spec.md                 # Requirements
│       ├── plan.md                 # Implementation plan
│       ├── tasks.md                # Task breakdown (75 tasks)
│       └── data-model.md           # Data structures
├── history/
│   └── prompts/                    # Development history (PHRs)
└── .specify/
    └── memory/
        └── constitution.md         # Project principles
```

---

## 🌐 Deployment

### GitHub Pages (Automated)

```bash
# Deploy to GitHub Pages
GIT_USER=<YOUR_GITHUB_USERNAME> npm run deploy
```

**Note:** Ensure GitHub Pages is enabled in repository settings:
1. Go to **Settings** → **Pages**
2. Source: `gh-pages` branch
3. Your site will be live at: `https://<username>.github.io/<repo-name>/`

### Vercel (Alternative)

1. Import repository to Vercel
2. Root directory: `physical-ai-textbook`
3. Build command: `npm run build`
4. Output directory: `build`

---

## 🎨 Features Implemented

### ✅ Phase 1-3 (Complete)
- Docusaurus initialization and configuration
- Directory structure and sidebar navigation
- Custom React components (ModuleCard, QuickLinksPanel)
- Homepage dashboard with module cards
- Module overview pages for all 4 modules

### ✅ Phase 4 (60% Complete)
- Hardware setup guides (Digital Twin, Jetson, Cloud)
- Software installation guides (ROS 2, Isaac Sim)
- Setup index with comparison tables
- Cost breakdowns and decision trees

### ⏳ In Progress
- Glossary search component (Phase 4)
- Detailed chapter content (Phase 5-8)
- Assessment rubrics and guidelines
- Troubleshooting guide expansion

---

## 📊 Progress Status

```
Overall: 26/75 tasks complete (35%)

Phase 1: ████████████████████ 100% (Setup)
Phase 2: ████████████████████ 100% (Infrastructure)
Phase 3: ████████████████████ 100% (Navigation)
Phase 4: ████████████░░░░░░░░  60% (Content)
Phase 5-8:                       0% (Pending)
```

**User Stories Completed:**
- ✅ US1: Navigate Complete Course Structure (100%)
- ⏳ US2: Foundational Setup (60%)
- ⏳ US3-5: Module Learning Path, Assessments, References (0%)

---

## 🤝 Contributing

Contributions are welcome! If you'd like to improve the Physical AI Textbook:

1. **Fork** the repository
2. Create a **feature branch** (`git checkout -b feature/amazing-content`)
3. **Commit** your changes (`git commit -m 'Add amazing content'`)
4. **Push** to the branch (`git push origin feature/amazing-content`)
5. Open a **Pull Request**

### Development Workflow

This project follows **Spec-Driven Development (SDD)** using SpecKit Plus:

1. `/sp.specify` - Create feature specification
2. `/sp.plan` - Generate implementation plan
3. `/sp.tasks` - Break down into tasks
4. `/sp.implement` - Execute tasks

All specifications are in `specs/` directory.

---

## 📝 Documentation Standards

### Content Guidelines

- **Frontmatter Required:** All markdown files must include title, description, estimated_time, week, module
- **Code Examples:** Must be tested and include dependency versions
- **File Naming:** kebab-case for markdown, PascalCase for React components
- **Images:** SVG preferred, PNG <200KB
- **Links:** Use relative paths, no broken links allowed

### Constitution Principles

See `.specify/memory/constitution.md` for 8 core principles:
1. Content Accuracy & Technical Rigor
2. Educational Clarity & Accessibility
3. Consistency & Standards
4. Docusaurus Structure & Quality
5. Code Example Quality
6. UI/UX Excellence
7. Deployment & Publishing
8. AI-Driven Content Standards

---

## 🎯 Roadmap

### Q1 2026
- [ ] Complete all 75 implementation tasks
- [ ] Add Glossary search component (Fuse.js integration)
- [ ] Expand chapter content for Modules 2-4
- [ ] Create interactive code sandboxes

### Q2 2026
- [ ] Add video tutorials (embedded YouTube)
- [ ] Implement progress tracking
- [ ] Community forum integration
- [ ] Multi-language support (Urdu, Arabic)

### Q3 2026
- [ ] Physical hardware integration guides
- [ ] Student project showcase
- [ ] Instructor dashboard
- [ ] LMS integration (Moodle/Canvas)

---

## 🏆 Hardware Options

Students can choose from 3 hardware setups:

| Option | Cost | Best For |
|--------|------|----------|
| **Digital Twin Workstation** | $1,530-2,450 | Maximum performance, offline work |
| **Jetson Orin Nano + Cloud** | $799-899 | Edge deployment, portability |
| **Cloud GPU Only** | $200-600 | No upfront cost, remote learning |

Full comparison available at: [Hardware Setup Guide](https://naveed261.github.io/GIAIC-HACKATON-1-PROJECT-1/docs/setup)

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

## 👨‍💻 Author

**NAVEED261**
- GitHub: [@NAVEED261](https://github.com/NAVEED261)
- Repository: [GIAIC-HACKATON-1-PROJECT-1](https://github.com/NAVEED261/GIAIC-HACKATON-1-PROJECT-1)

---

## 🙏 Acknowledgments

- **NVIDIA** for Isaac Sim and Omniverse platform
- **Open Robotics** for ROS 2 ecosystem
- **Docusaurus** team for amazing documentation framework
- **GIAIC Hackathon** for project inspiration

---

## 📞 Support

- **Issues:** [GitHub Issues](https://github.com/NAVEED261/GIAIC-HACKATON-1-PROJECT-1/issues)
- **Discussions:** [GitHub Discussions](https://github.com/NAVEED261/GIAIC-HACKATON-1-PROJECT-1/discussions)
- **Website:** [Physical AI Textbook](https://naveed261.github.io/GIAIC-HACKATON-1-PROJECT-1/)

---

## ⭐ Star this Repository

If you find this project helpful, please consider giving it a star! ⭐

---

**Built with 🤖 by Claude Code | Powered by Docusaurus v3 | Deployed on GitHub Pages**

Last Updated: December 9, 2025
