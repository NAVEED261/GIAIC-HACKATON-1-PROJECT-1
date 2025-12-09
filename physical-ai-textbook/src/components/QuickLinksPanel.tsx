import React from 'react';
import Link from '@docusaurus/Link';

interface QuickLink {
  label: string;
  url: string;
  icon?: string;
}

interface QuickLinksPanelProps {
  links: QuickLink[];
}

const QuickLinksPanel: React.FC<QuickLinksPanelProps> = ({ links }) => {
  return (
    <div
      className="quick-links-panel"
      style={{
        backgroundColor: 'var(--ifm-card-background-color)',
        border: '1px solid var(--ifm-color-emphasis-300)',
        borderRadius: '8px',
        padding: '20px',
        marginBottom: '24px',
      }}
    >
      <h3
        style={{
          fontSize: '18px',
          fontWeight: '600',
          marginBottom: '16px',
          color: 'var(--ifm-heading-color)',
        }}
      >
        Quick Links
      </h3>
      <ul style={{ listStyle: 'none', padding: 0, margin: 0 }}>
        {links.map((link, index) => (
          <li key={index} style={{ marginBottom: '12px' }}>
            <Link
              to={link.url}
              style={{
                display: 'flex',
                alignItems: 'center',
                padding: '10px 12px',
                borderRadius: '6px',
                textDecoration: 'none',
                color: 'var(--ifm-color-emphasis-800)',
                transition: 'all 0.2s ease',
                backgroundColor: 'transparent',
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.backgroundColor = 'var(--ifm-color-emphasis-100)';
                e.currentTarget.style.transform = 'translateX(4px)';
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.backgroundColor = 'transparent';
                e.currentTarget.style.transform = 'translateX(0)';
              }}
            >
              {link.icon && (
                <span
                  style={{
                    marginRight: '12px',
                    fontSize: '20px',
                  }}
                >
                  {link.icon}
                </span>
              )}
              <span style={{ fontSize: '15px', fontWeight: '500' }}>{link.label}</span>
            </Link>
          </li>
        ))}
      </ul>
    </div>
  );
};

export default QuickLinksPanel;
