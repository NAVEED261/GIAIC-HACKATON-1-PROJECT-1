# Docusaurus Chatbot Widget Agent Skill

**Purpose:** Expert in integrating React chat widget into Docusaurus with text selection, API calls, and UI/UX best practices.

## Expertise

- React 18 with TypeScript
- Docusaurus v3 custom components
- Text selection API (browser)
- Fetch API for backend communication
- CSS modules for styling
- Dark/light mode support
- Mobile-responsive design
- Loading states and error handling

## Component Architecture

```
physical-ai-textbook/src/components/
├── ChatWidget/
│   ├── ChatWidget.tsx           # Main widget component
│   ├── ChatMessage.tsx          # Message bubble component
│   ├── ChatInput.tsx            # Input field with send button
│   ├── ChatHistory.tsx          # Message list
│   ├── ChatWidget.module.css    # Scoped styles
│   └── types.ts                 # TypeScript interfaces
```

## Code Patterns

### 1. Main Chat Widget Component

```typescript
// src/components/ChatWidget/ChatWidget.tsx
import React, { useState, useEffect, useRef } from 'react';
import styles from './ChatWidget.module.css';
import ChatHistory from './ChatHistory';
import ChatInput from './ChatInput';
import { Message } from './types';

interface ChatWidgetProps {
  apiUrl?: string;
}

const ChatWidget: React.FC<ChatWidgetProps> = ({
  apiUrl = 'https://your-backend.railway.app/api/v1'
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId] = useState(() => `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`);
  const [selectedText, setSelectedText] = useState<string>('');

  // Text selection handler
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 10) {
        setSelectedText(text);
        setIsOpen(true);  // Auto-open widget with selection
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const sendMessage = async (userMessage: string) => {
    // Add user message to UI
    const newMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: userMessage,
      timestamp: new Date().toISOString()
    };
    setMessages(prev => [...prev, newMessage]);
    setIsLoading(true);

    try {
      // Call backend API
      const response = await fetch(`${apiUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          query: userMessage,
          session_id: sessionId,
          selected_text: selectedText || null
        })
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant response
      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.answer,
        sources: data.sources,
        confidence: data.confidence,
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, assistantMessage]);
      setSelectedText('');  // Clear selected text
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* Floating button */}
      <button
        className={styles.floatingButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Open chatbot"
      >
        💬
      </button>

      {/* Chat panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          <div className={styles.chatHeader}>
            <h3>Physical AI Assistant</h3>
            <button onClick={() => setIsOpen(false)} aria-label="Close">×</button>
          </div>

          {selectedText && (
            <div className={styles.selectedTextBanner}>
              <span>Selected: "{selectedText.slice(0, 50)}..."</span>
              <button onClick={() => setSelectedText('')}>×</button>
            </div>
          )}

          <ChatHistory messages={messages} isLoading={isLoading} />
          <ChatInput onSend={sendMessage} disabled={isLoading} />
        </div>
      )}
    </>
  );
};

export default ChatWidget;
```

### 2. TypeScript Interfaces

```typescript
// src/components/ChatWidget/types.ts
export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  confidence?: number;
  timestamp: string;
}

export interface Source {
  chapter: string;
  score: number;
  module?: number;
  week?: number;
}

export interface ChatRequest {
  query: string;
  session_id: string;
  selected_text?: string | null;
}

export interface ChatResponse {
  answer: string;
  sources: Source[];
  session_id: string;
  confidence: number;
}
```

### 3. Chat History Component

```typescript
// src/components/ChatWidget/ChatHistory.tsx
import React, { useEffect, useRef } from 'react';
import ChatMessage from './ChatMessage';
import { Message } from './types';
import styles from './ChatWidget.module.css';

interface ChatHistoryProps {
  messages: Message[];
  isLoading: boolean;
}

const ChatHistory: React.FC<ChatHistoryProps> = ({ messages, isLoading }) => {
  const endOfMessagesRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom
  useEffect(() => {
    endOfMessagesRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <div className={styles.chatHistory}>
      {messages.length === 0 && (
        <div className={styles.emptyState}>
          <p>👋 Hi! I'm your Physical AI teaching assistant.</p>
          <p>Ask me about ROS 2, Isaac Sim, or select text to get context-aware answers.</p>
        </div>
      )}

      {messages.map(message => (
        <ChatMessage key={message.id} message={message} />
      ))}

      {isLoading && (
        <div className={styles.loadingIndicator}>
          <span className={styles.dot}></span>
          <span className={styles.dot}></span>
          <span className={styles.dot}></span>
        </div>
      )}

      <div ref={endOfMessagesRef} />
    </div>
  );
};

export default ChatHistory;
```

### 4. Chat Message Component

```typescript
// src/components/ChatWidget/ChatMessage.tsx
import React from 'react';
import { Message } from './types';
import styles from './ChatWidget.module.css';

interface ChatMessageProps {
  message: Message;
}

const ChatMessage: React.FC<ChatMessageProps> = ({ message }) => {
  const isUser = message.role === 'user';

  return (
    <div className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage}`}>
      <div className={styles.messageContent}>
        {message.content}
      </div>

      {/* Show sources for assistant messages */}
      {!isUser && message.sources && message.sources.length > 0 && (
        <div className={styles.sources}>
          <p className={styles.sourcesLabel}>Sources:</p>
          {message.sources.map((source, idx) => (
            <span key={idx} className={styles.sourceTag}>
              {source.chapter} (score: {(source.score * 100).toFixed(0)}%)
            </span>
          ))}
        </div>
      )}

      {/* Show confidence */}
      {!isUser && message.confidence !== undefined && (
        <div className={styles.confidence}>
          Confidence: {(message.confidence * 100).toFixed(0)}%
        </div>
      )}
    </div>
  );
};

export default ChatMessage;
```

### 5. Chat Input Component

```typescript
// src/components/ChatWidget/ChatInput.tsx
import React, { useState, KeyboardEvent } from 'react';
import styles from './ChatWidget.module.css';

interface ChatInputProps {
  onSend: (message: string) => void;
  disabled: boolean;
}

const ChatInput: React.FC<ChatInputProps> = ({ onSend, disabled }) => {
  const [input, setInput] = useState('');

  const handleSend = () => {
    if (input.trim() && !disabled) {
      onSend(input.trim());
      setInput('');
    }
  };

  const handleKeyPress = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <div className={styles.chatInput}>
      <textarea
        value={input}
        onChange={(e) => setInput(e.target.value)}
        onKeyPress={handleKeyPress}
        placeholder="Ask about ROS 2, Isaac Sim, or select text..."
        disabled={disabled}
        rows={2}
      />
      <button onClick={handleSend} disabled={disabled || !input.trim()}>
        Send
      </button>
    </div>
  );
};

export default ChatInput;
```

### 6. CSS Styling (Dark/Light Mode)

```css
/* src/components/ChatWidget/ChatWidget.module.css */
.floatingButton {
  position: fixed;
  bottom: 20px;
  right: 20px;
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  font-size: 24px;
  cursor: pointer;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  z-index: 1000;
  transition: transform 0.2s;
}

.floatingButton:hover {
  transform: scale(1.1);
}

.chatPanel {
  position: fixed;
  bottom: 90px;
  right: 20px;
  width: 400px;
  height: 600px;
  background: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 12px;
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.15);
  display: flex;
  flex-direction: column;
  z-index: 1000;
}

.chatHeader {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px;
  background: var(--ifm-color-primary);
  color: white;
  border-radius: 12px 12px 0 0;
}

.chatHistory {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
}

.message {
  margin-bottom: 12px;
  padding: 12px;
  border-radius: 8px;
  max-width: 85%;
}

.userMessage {
  background: var(--ifm-color-primary);
  color: white;
  margin-left: auto;
}

.assistantMessage {
  background: var(--ifm-color-emphasis-100);
  color: var(--ifm-font-color-base);
}

.chatInput {
  display: flex;
  gap: 8px;
  padding: 16px;
  border-top: 1px solid var(--ifm-color-emphasis-300);
}

.chatInput textarea {
  flex: 1;
  padding: 8px;
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  resize: none;
  font-family: inherit;
  background: var(--ifm-background-color);
  color: var(--ifm-font-color-base);
}

.chatInput button {
  padding: 8px 16px;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  border-radius: 8px;
  cursor: pointer;
}

/* Mobile responsive */
@media (max-width: 768px) {
  .chatPanel {
    width: calc(100vw - 40px);
    height: calc(100vh - 120px);
    right: 20px;
    bottom: 80px;
  }
}
```

### 7. Embed in Docusaurus Layout

```typescript
// src/theme/Root.tsx
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget apiUrl={process.env.REACT_APP_API_URL || 'http://localhost:8000/api/v1'} />
    </>
  );
}
```

## Text Selection Integration

```typescript
// Enhanced text selection with context
const handleSelection = () => {
  const selection = window.getSelection();
  const text = selection?.toString().trim();

  if (text && text.length > 10) {
    // Get surrounding context (paragraph)
    const range = selection.getRangeAt(0);
    const container = range.commonAncestorContainer;
    const paragraph = container.nodeType === 3
      ? container.parentElement?.textContent
      : container.textContent;

    setSelectedText(text);
    setContextParagraph(paragraph || '');
    setIsOpen(true);
  }
};
```

## Best Practices

1. **Session ID generation** - Use timestamp + random for uniqueness
2. **Auto-scroll to bottom** - useRef + useEffect for smooth UX
3. **Loading states** - Show typing indicator during API calls
4. **Error handling** - Display user-friendly error messages
5. **Dark mode support** - Use Docusaurus CSS variables
6. **Mobile responsive** - Adjust width/height for small screens
7. **Accessibility** - ARIA labels for buttons
8. **Text selection UX** - Auto-open widget with selected text

## Testing

```typescript
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import ChatWidget from './ChatWidget';

test('sends message on button click', async () => {
  render(<ChatWidget />);

  const input = screen.getByPlaceholderText(/Ask about/i);
  const sendButton = screen.getByText(/Send/i);

  fireEvent.change(input, { target: { value: 'What is ROS 2?' } });
  fireEvent.click(sendButton);

  await waitFor(() => {
    expect(screen.getByText('What is ROS 2?')).toBeInTheDocument();
  });
});
```

## When to Use This Agent

- Building React chat UI components
- Integrating chat widget into Docusaurus
- Implementing text selection handlers
- Styling with CSS modules (dark/light mode)
- Testing React components with Testing Library
- Optimizing mobile responsiveness
