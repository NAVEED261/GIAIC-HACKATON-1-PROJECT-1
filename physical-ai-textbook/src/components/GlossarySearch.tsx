import React, { useState, useMemo } from 'react';
import Fuse from 'fuse.js';

interface GlossaryTerm {
  term: string;
  aliases?: string[];
  definition: string;
  relatedChapters?: string[];
  externalLinks?: { label: string; url: string }[];
}

interface GlossarySearchProps {
  terms: GlossaryTerm[];
}

const GlossarySearch: React.FC<GlossarySearchProps> = ({ terms }) => {
  const [searchQuery, setSearchQuery] = useState('');
  const [selectedLetter, setSelectedLetter] = useState<string | null>(null);

  // Configure Fuse.js for fuzzy search
  const fuse = useMemo(
    () =>
      new Fuse(terms, {
        keys: ['term', 'aliases', 'definition'],
        threshold: 0.3, // Fuzzy matching threshold (0 = exact, 1 = match anything)
        includeScore: true,
        minMatchCharLength: 2,
      }),
    [terms]
  );

  // Perform search
  const searchResults = useMemo(() => {
    if (!searchQuery.trim()) {
      // No search query - filter by selected letter or show all
      if (selectedLetter) {
        return terms.filter((term) =>
          term.term.toUpperCase().startsWith(selectedLetter)
        );
      }
      return terms;
    }

    // Perform fuzzy search
    const results = fuse.search(searchQuery);
    return results.map((result) => result.item);
  }, [searchQuery, selectedLetter, terms, fuse]);

  // Get alphabet for letter filtering
  const alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'.split('');

  // Get available first letters from terms
  const availableLetters = useMemo(() => {
    const letters = new Set(
      terms.map((term) => term.term.charAt(0).toUpperCase())
    );
    return alphabet.filter((letter) => letters.has(letter));
  }, [terms]);

  return (
    <div style={{ marginTop: '20px' }}>
      {/* Search Input */}
      <div style={{ marginBottom: '24px' }}>
        <input
          type="text"
          placeholder="Search for terms, definitions, or aliases..."
          value={searchQuery}
          onChange={(e) => {
            setSearchQuery(e.target.value);
            setSelectedLetter(null); // Clear letter filter when searching
          }}
          style={{
            width: '100%',
            padding: '12px 16px',
            fontSize: '16px',
            border: '2px solid var(--ifm-color-emphasis-300)',
            borderRadius: '8px',
            outline: 'none',
            transition: 'border-color 0.2s',
          }}
          onFocus={(e) => {
            e.target.style.borderColor = 'var(--ifm-color-primary)';
          }}
          onBlur={(e) => {
            e.target.style.borderColor = 'var(--ifm-color-emphasis-300)';
          }}
        />
        {searchQuery && (
          <div style={{ marginTop: '8px', fontSize: '14px', color: 'var(--ifm-color-emphasis-700)' }}>
            Found {searchResults.length} result{searchResults.length !== 1 ? 's' : ''}
          </div>
        )}
      </div>

      {/* Letter Filter (only show when not searching) */}
      {!searchQuery && (
        <div
          style={{
            display: 'flex',
            flexWrap: 'wrap',
            gap: '8px',
            marginBottom: '24px',
            padding: '12px',
            backgroundColor: 'var(--ifm-background-surface-color)',
            borderRadius: '8px',
          }}
        >
          <button
            onClick={() => setSelectedLetter(null)}
            style={{
              padding: '6px 12px',
              border: selectedLetter === null ? '2px solid var(--ifm-color-primary)' : '1px solid var(--ifm-color-emphasis-300)',
              borderRadius: '4px',
              backgroundColor: selectedLetter === null ? 'var(--ifm-color-primary-lighter)' : 'transparent',
              color: selectedLetter === null ? 'var(--ifm-color-primary-darkest)' : 'inherit',
              cursor: 'pointer',
              fontSize: '14px',
              fontWeight: selectedLetter === null ? 'bold' : 'normal',
            }}
          >
            All
          </button>
          {availableLetters.map((letter) => (
            <button
              key={letter}
              onClick={() => setSelectedLetter(letter)}
              style={{
                padding: '6px 12px',
                border: selectedLetter === letter ? '2px solid var(--ifm-color-primary)' : '1px solid var(--ifm-color-emphasis-300)',
                borderRadius: '4px',
                backgroundColor: selectedLetter === letter ? 'var(--ifm-color-primary-lighter)' : 'transparent',
                color: selectedLetter === letter ? 'var(--ifm-color-primary-darkest)' : 'inherit',
                cursor: 'pointer',
                fontSize: '14px',
                fontWeight: selectedLetter === letter ? 'bold' : 'normal',
              }}
            >
              {letter}
            </button>
          ))}
        </div>
      )}

      {/* Search Results / Term List */}
      <div>
        {searchResults.length === 0 ? (
          <div
            style={{
              padding: '40px',
              textAlign: 'center',
              color: 'var(--ifm-color-emphasis-600)',
            }}
          >
            No terms found matching "{searchQuery}"
          </div>
        ) : (
          searchResults.map((term, index) => (
            <div
              key={index}
              style={{
                marginBottom: '24px',
                padding: '16px',
                border: '1px solid var(--ifm-color-emphasis-200)',
                borderRadius: '8px',
                backgroundColor: 'var(--ifm-card-background-color)',
              }}
            >
              {/* Term */}
              <h3
                style={{
                  margin: '0 0 8px 0',
                  fontSize: '20px',
                  color: 'var(--ifm-color-primary)',
                }}
              >
                {term.term}
              </h3>

              {/* Aliases */}
              {term.aliases && term.aliases.length > 0 && (
                <div
                  style={{
                    marginBottom: '8px',
                    fontSize: '14px',
                    color: 'var(--ifm-color-emphasis-700)',
                    fontStyle: 'italic',
                  }}
                >
                  Also known as: {term.aliases.join(', ')}
                </div>
              )}

              {/* Definition */}
              <p
                style={{
                  margin: '12px 0',
                  lineHeight: '1.6',
                  color: 'var(--ifm-font-color-base)',
                }}
              >
                {term.definition}
              </p>

              {/* Related Chapters */}
              {term.relatedChapters && term.relatedChapters.length > 0 && (
                <div style={{ marginTop: '12px' }}>
                  <strong style={{ fontSize: '14px' }}>Related chapters:</strong>
                  <ul style={{ margin: '4px 0 0 0', paddingLeft: '20px' }}>
                    {term.relatedChapters.map((chapter, idx) => (
                      <li key={idx} style={{ fontSize: '14px' }}>
                        {chapter}
                      </li>
                    ))}
                  </ul>
                </div>
              )}

              {/* External Links */}
              {term.externalLinks && term.externalLinks.length > 0 && (
                <div style={{ marginTop: '12px' }}>
                  <strong style={{ fontSize: '14px' }}>Learn more:</strong>
                  <ul style={{ margin: '4px 0 0 0', paddingLeft: '20px' }}>
                    {term.externalLinks.map((link, idx) => (
                      <li key={idx} style={{ fontSize: '14px' }}>
                        <a
                          href={link.url}
                          target="_blank"
                          rel="noopener noreferrer"
                          style={{ color: 'var(--ifm-color-primary)' }}
                        >
                          {link.label}
                        </a>
                      </li>
                    ))}
                  </ul>
                </div>
              )}
            </div>
          ))
        )}
      </div>
    </div>
  );
};

export default GlossarySearch;
