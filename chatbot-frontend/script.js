// Configuration
const API_BASE_URL = 'https://giaic-hackaton-1-project-1.onrender.com/api/v1';
const SESSION_ID = 'session-' + Date.now() + '-' + Math.random().toString(36).substr(2, 9);

// DOM Elements
const chatMessages = document.getElementById('chatMessages');
const userInput = document.getElementById('userInput');
const sendButton = document.getElementById('sendButton');
const sendIcon = document.getElementById('sendIcon');
const loadingIcon = document.getElementById('loadingIcon');

// Initialize
document.addEventListener('DOMContentLoaded', () => {
    userInput.focus();

    // Send message on button click
    sendButton.addEventListener('click', sendMessage);

    // Send message on Enter key
    userInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendMessage();
        }
    });
});

// Send Message Function
async function sendMessage() {
    const message = userInput.value.trim();

    if (!message) return;

    // Disable input
    setLoading(true);

    // Add user message to chat
    addMessage(message, 'user');

    // Clear input
    userInput.value = '';

    // Show typing indicator
    const typingIndicator = showTypingIndicator();

    try {
        // Call API
        const response = await fetch(`${API_BASE_URL}/chat`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                query: message,
                session_id: SESSION_ID
            })
        });

        if (!response.ok) {
            throw new Error(`API Error: ${response.status} ${response.statusText}`);
        }

        const data = await response.json();

        // Remove typing indicator
        typingIndicator.remove();

        // Add bot response
        addMessage(data.answer, 'bot', data.sources, data.confidence);

    } catch (error) {
        console.error('Error:', error);

        // Remove typing indicator
        typingIndicator.remove();

        // Show error message
        addErrorMessage(error.message);
    } finally {
        // Re-enable input
        setLoading(false);
        userInput.focus();
    }
}

// Add Message to Chat
function addMessage(text, sender, sources = null, confidence = null) {
    const messageDiv = document.createElement('div');
    messageDiv.className = `message ${sender}-message`;

    const contentDiv = document.createElement('div');
    contentDiv.className = 'message-content';

    const textP = document.createElement('p');
    textP.textContent = text;
    contentDiv.appendChild(textP);

    // Add sources if available
    if (sources && sources.length > 0) {
        const sourcesDiv = document.createElement('div');
        sourcesDiv.className = 'sources';

        const sourcesTitle = document.createElement('strong');
        sourcesTitle.textContent = '📚 Sources:';
        sourcesDiv.appendChild(sourcesTitle);

        sources.forEach(source => {
            const sourceSpan = document.createElement('span');
            sourceSpan.className = 'source-item';
            sourceSpan.textContent = `${source.chapter || 'Unknown'} (Week ${source.week || 'N/A'})`;
            sourcesDiv.appendChild(sourceSpan);
        });

        contentDiv.appendChild(sourcesDiv);
    }

    // Add confidence if available
    if (confidence !== null) {
        const confidenceP = document.createElement('p');
        confidenceP.style.fontSize = '12px';
        confidenceP.style.color = '#666';
        confidenceP.style.marginTop = '8px';
        confidenceP.textContent = `Confidence: ${(confidence * 100).toFixed(1)}%`;
        contentDiv.appendChild(confidenceP);
    }

    messageDiv.appendChild(contentDiv);
    chatMessages.appendChild(messageDiv);

    // Scroll to bottom
    scrollToBottom();
}

// Show Typing Indicator
function showTypingIndicator() {
    const messageDiv = document.createElement('div');
    messageDiv.className = 'message bot-message';
    messageDiv.id = 'typing-indicator';

    const contentDiv = document.createElement('div');
    contentDiv.className = 'message-content';

    const typingDiv = document.createElement('div');
    typingDiv.className = 'typing-indicator';
    typingDiv.innerHTML = '<span></span><span></span><span></span>';

    contentDiv.appendChild(typingDiv);
    messageDiv.appendChild(contentDiv);
    chatMessages.appendChild(messageDiv);

    scrollToBottom();

    return messageDiv;
}

// Add Error Message
function addErrorMessage(errorText) {
    const errorDiv = document.createElement('div');
    errorDiv.className = 'error-message';
    errorDiv.innerHTML = `
        <strong>⚠️ Error:</strong>
        <p>${errorText}</p>
        <p style="margin-top: 8px; font-size: 12px;">
            Please check your connection or try again later.
        </p>
    `;

    chatMessages.appendChild(errorDiv);
    scrollToBottom();
}

// Set Loading State
function setLoading(isLoading) {
    userInput.disabled = isLoading;
    sendButton.disabled = isLoading;

    if (isLoading) {
        sendIcon.classList.add('hidden');
        loadingIcon.classList.remove('hidden');
    } else {
        sendIcon.classList.remove('hidden');
        loadingIcon.classList.add('hidden');
    }
}

// Scroll to Bottom
function scrollToBottom() {
    chatMessages.scrollTop = chatMessages.scrollHeight;
}

// Example Questions (Optional)
const exampleQuestions = [
    "What is ROS 2?",
    "Explain digital twins in robotics",
    "How does NVIDIA Isaac Sim work?",
    "What is a VLA in robotics?"
];

// You can add quick question buttons if needed
function addQuickQuestions() {
    const quickQuestionsDiv = document.createElement('div');
    quickQuestionsDiv.className = 'quick-questions';
    quickQuestionsDiv.innerHTML = '<p><strong>Try asking:</strong></p>';

    exampleQuestions.forEach(question => {
        const button = document.createElement('button');
        button.textContent = question;
        button.className = 'quick-question-btn';
        button.onclick = () => {
            userInput.value = question;
            sendMessage();
        };
        quickQuestionsDiv.appendChild(button);
    });

    chatMessages.appendChild(quickQuestionsDiv);
}
