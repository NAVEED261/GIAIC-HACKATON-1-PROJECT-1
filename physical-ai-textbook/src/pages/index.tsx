import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import ModuleCard from '@site/src/components/ModuleCard';
import QuickLinksPanel from '@site/src/components/QuickLinksPanel';

import styles from './index.module.css';
import modulesData from '@site/static/data/modules.json';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning →
          </Link>
          <Link
            className="button button--outline button--secondary button--lg"
            to="/docs/setup"
            style={{ marginLeft: '12px' }}>
            Hardware Setup
          </Link>
        </div>
      </div>
    </header>
  );
}

function ModulesDashboard() {
  const quickLinks = [
    { label: 'Hardware Setup', url: '/docs/setup', icon: '⚙️' },
    { label: 'Assessments', url: '/docs/assessments', icon: '📝' },
    { label: 'Glossary', url: '/docs/reference/glossary', icon: '📖' },
    { label: 'Troubleshooting', url: '/docs/reference/troubleshooting', icon: '🔧' },
  ];

  return (
    <section className="container" style={{ marginTop: '48px', marginBottom: '64px' }}>
      <div className="row">
        {/* Main Content - Module Cards */}
        <div className="col col--9">
          <div style={{ marginBottom: '32px' }}>
            <Heading as="h2">13-Week Course Modules</Heading>
            <p style={{ fontSize: '16px', color: 'var(--ifm-color-emphasis-700)' }}>
              Master Physical AI through four comprehensive modules: ROS 2 fundamentals, digital
              twin simulation, NVIDIA Isaac Sim, and vision-language-action models for humanoid
              robotics.
            </p>
          </div>

          {/* Module Cards Grid */}
          <div
            style={{
              display: 'grid',
              gridTemplateColumns: 'repeat(auto-fit, minmax(300px, 1fr))',
              gap: '24px',
              marginBottom: '48px',
            }}
          >
            {modulesData.map((module) => (
              <ModuleCard
                key={module.id}
                moduleNumber={module.moduleNumber}
                title={module.title}
                weekRange={module.weekRange}
                learningOutcomes={module.learningOutcomes}
                estimatedHours={module.estimatedHours}
                link={`/docs/${module.id}`}
                color={module.color}
              />
            ))}
          </div>

          {/* Course Information */}
          <div
            style={{
              backgroundColor: 'var(--ifm-card-background-color)',
              border: '1px solid var(--ifm-color-emphasis-300)',
              borderRadius: '8px',
              padding: '24px',
            }}
          >
            <Heading as="h3" style={{ fontSize: '20px', marginBottom: '16px' }}>
              Course Overview
            </Heading>
            <div className="row">
              <div className="col col--4">
                <div style={{ textAlign: 'center', padding: '16px' }}>
                  <div style={{ fontSize: '32px', fontWeight: 'bold', color: 'var(--ifm-color-primary)' }}>
                    13
                  </div>
                  <div style={{ fontSize: '14px', color: 'var(--ifm-color-emphasis-700)' }}>Weeks</div>
                </div>
              </div>
              <div className="col col--4">
                <div style={{ textAlign: 'center', padding: '16px' }}>
                  <div style={{ fontSize: '32px', fontWeight: 'bold', color: 'var(--ifm-color-primary)' }}>
                    110+
                  </div>
                  <div style={{ fontSize: '14px', color: 'var(--ifm-color-emphasis-700)' }}>Hours</div>
                </div>
              </div>
              <div className="col col--4">
                <div style={{ textAlign: 'center', padding: '16px' }}>
                  <div style={{ fontSize: '32px', fontWeight: 'bold', color: 'var(--ifm-color-primary)' }}>
                    4
                  </div>
                  <div style={{ fontSize: '14px', color: 'var(--ifm-color-emphasis-700)' }}>Modules</div>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Sidebar - Quick Links */}
        <div className="col col--3">
          <QuickLinksPanel links={quickLinks} />

          {/* Additional Info Box */}
          <div
            style={{
              backgroundColor: 'var(--ifm-color-primary-lightest)',
              border: '1px solid var(--ifm-color-primary-light)',
              borderRadius: '8px',
              padding: '20px',
            }}
          >
            <Heading as="h4" style={{ fontSize: '16px', marginBottom: '12px' }}>
              🎯 Capstone Project
            </Heading>
            <p style={{ fontSize: '14px', margin: 0, color: 'var(--ifm-color-emphasis-800)' }}>
              Build an autonomous humanoid robot with voice interface, task planning, navigation,
              perception, and manipulation capabilities.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="13-Week Physical AI Course: Learn ROS 2, Digital Twin Simulation, NVIDIA Isaac Sim, and VLA Integration for Humanoid Robotics">
      <HomepageHeader />
      <main>
        <ModulesDashboard />
      </main>
    </Layout>
  );
}
