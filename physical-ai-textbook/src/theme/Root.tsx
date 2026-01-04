import React from 'react';
import ChatbotWidget from '../components/ChatbotWidget';
import { AuthProvider } from '../contexts/AuthContext';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <AuthProvider>
      <>
        {children}
        <ChatbotWidget />
      </>
    </AuthProvider>
  );
}
