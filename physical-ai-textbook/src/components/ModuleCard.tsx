import React from 'react';
import Link from '@docusaurus/Link';

interface ModuleCardProps {
  moduleNumber: number;
  title: string;
  weekRange: string;
  learningOutcomes: string[];
  estimatedHours: number;
  link: string;
  color?: string;
}

const ModuleCard: React.FC<ModuleCardProps> = ({
  moduleNumber,
  title,
  weekRange,
  learningOutcomes,
  estimatedHours,
  link,
  color = '#3578e5',
}) => {
  return (
    <div
      className="module-card"
      style={{
        border: `2px solid ${color}`,
        borderRadius: '12px',
        padding: '24px',
        height: '100%',
        display: 'flex',
        flexDirection: 'column',
        transition: 'all 0.3s ease',
        backgroundColor: 'var(--ifm-card-background-color)',
      }}
      onMouseEnter={(e) => {
        e.currentTarget.style.boxShadow = `0 4px 16px ${color}40`;
        e.currentTarget.style.transform = 'translateY(-4px)';
      }}
      onMouseLeave={(e) => {
        e.currentTarget.style.boxShadow = 'none';
        e.currentTarget.style.transform = 'translateY(0)';
      }}
    >
      {/* Module Badge */}
      <div
        style={{
          display: 'inline-block',
          backgroundColor: color,
          color: 'white',
          padding: '6px 12px',
          borderRadius: '6px',
          fontSize: '14px',
          fontWeight: 'bold',
          marginBottom: '16px',
          alignSelf: 'flex-start',
        }}
      >
        Module {moduleNumber}
      </div>

      {/* Title */}
      <h3
        style={{
          color: color,
          marginBottom: '8px',
          fontSize: '24px',
          fontWeight: '600',
        }}
      >
        {title}
      </h3>

      {/* Week Range */}
      <p
        style={{
          color: 'var(--ifm-color-emphasis-600)',
          marginBottom: '16px',
          fontSize: '14px',
          fontWeight: '500',
        }}
      >
        {weekRange} • {estimatedHours} hours
      </p>

      {/* Learning Outcomes */}
      <div style={{ marginBottom: '20px', flexGrow: 1 }}>
        <h4 style={{ fontSize: '16px', marginBottom: '12px', color: 'var(--ifm-heading-color)' }}>
          Learning Outcomes
        </h4>
        <ul style={{ paddingLeft: '20px', margin: 0 }}>
          {learningOutcomes.slice(0, 3).map((outcome, index) => (
            <li
              key={index}
              style={{
                fontSize: '14px',
                marginBottom: '8px',
                color: 'var(--ifm-color-emphasis-700)',
              }}
            >
              {outcome}
            </li>
          ))}
          {learningOutcomes.length > 3 && (
            <li
              style={{
                fontSize: '14px',
                color: 'var(--ifm-color-emphasis-600)',
                fontStyle: 'italic',
              }}
            >
              +{learningOutcomes.length - 3} more...
            </li>
          )}
        </ul>
      </div>

      {/* CTA Button */}
      <Link
        to={link}
        style={{
          backgroundColor: color,
          color: 'white',
          padding: '10px 20px',
          borderRadius: '6px',
          textDecoration: 'none',
          textAlign: 'center',
          fontWeight: '600',
          fontSize: '16px',
          transition: 'opacity 0.2s',
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.opacity = '0.9';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.opacity = '1';
        }}
      >
        Start Module →
      </Link>
    </div>
  );
};

export default ModuleCard;
