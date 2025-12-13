import React, { useState, useEffect, useRef, useMemo, useCallback } from 'react';
import styles from './styles.module.css';

const API_BASE_URL = 'https://giaic-hackaton-1-project-1.onrender.com/api/v1';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    chapter: string;
    week: number;
    module: number;
    score: number;
  }>;
  confidence?: number;
}

// Memoized message component for performance
const ChatMessage = React.memo(({ message }: { message: Message }) => (
  <div className={`${styles.message} ${styles[message.role]}`}>
    <p>{message.content}</p>
    {message.sources && message.sources.length > 0 && (
      <div className={styles.sources}>
        <small>📚 Sources: {message.sources.map(s => s.chapter).join(', ')}</small>
      </div>
    )}
    {message.confidence && (
      <div className={styles.confidence}>
        <small>⭐ Confidence: {(message.confidence * 100).toFixed(0)}%</small>
      </div>
    )}
  </div>
));

export default function ChatbotWidget(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      role: 'assistant',
      content: '👋 Hi! I\'m your Physical AI assistant. Ask me about ROS 2, Digital Twins, NVIDIA Isaac Sim, VLA Models, Humanoid Robotics, Sensors, or Motion Planning!'
    }
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const sessionId = useRef(`session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`);

  // Memoized messages list - prevents unnecessary re-renders
  const memoizedMessages = useMemo(() => messages, [messages]);

  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [memoizedMessages, scrollToBottom]);

  const sendMessage = useCallback(async () => {
    if (!input.trim() || isLoading) return;

    const userQuery = input;
    const userMessage: Message = { role: 'user', content: userQuery };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 10000); // 10s timeout

      const response = await fetch(`${API_BASE_URL}/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: userQuery,
          session_id: sessionId.current
        }),
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (!response.ok) throw new Error(`API Error: ${response.status}`);

      const data = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources,
        confidence: data.confidence
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Chat error:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: '⚠️ Error: Could not get response. Please try again.'
      }]);
    } finally {
      setIsLoading(false);
    }
  }, [input, isLoading]);

  const handleKeyPress = useCallback((e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  }, [sendMessage]);

  return (
    <>
      {/* Floating Button - Fast */}
      <button
        className={styles.floatingButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chatbot"
        title="Chat with AI Assistant"
      >
        {isOpen ? '✕' : '🤖'}
      </button>

      {/* Chat Widget - Optimized */}
      {isOpen && (
        <div className={styles.chatWidget}>
          {/* Header */}
          <div className={styles.header}>
            <h3>Physical AI Assistant</h3>
            <button
              className={styles.closeBtn}
              onClick={() => setIsOpen(false)}
              aria-label="Close"
            >
              ✕
            </button>
          </div>

          {/* Messages Container - Memoized */}
          <div className={styles.messagesContainer}>
            {memoizedMessages.map((msg, idx) => (
              <ChatMessage key={idx} message={msg} />
            ))}
            {isLoading && (
              <div className={styles.loadingIndicator}>
                <span>⏳ Thinking...</span>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Container */}
          <div className={styles.inputContainer}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about ROS 2, Digital Twins, Isaac Sim..."
              disabled={isLoading}
              className={styles.input}
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !input.trim()}
              className={styles.sendButton}
              aria-label="Send message"
            >
              {isLoading ? '⏳' : '➤'}
            </button>
          </div>

          {/* Status */}
          {isLoading && (
            <div className={styles.status}>
              <small>Getting answer...</small>
            </div>
          )}
        </div>
      )}
    </>
  );
}
