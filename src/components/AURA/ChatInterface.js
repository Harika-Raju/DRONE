import React, { useState, useRef, useEffect } from 'react';
import { GoogleGenerativeAI } from '@google/generative-ai';
import { Bot, Send, X, BatteryMedium, Signal, Cloud } from 'lucide-react';
import { AURA_SYSTEM_CONTEXT, generateEnhancedPrompt, getQueryType } from './AuraPrompts';
import './ChatInterface.css';

// At the top of ChatInterface.js, right after imports
const GEMINI_API_KEY = process.env.REACT_APP_GEMINI_API_KEY;


const ChatInterface = ({ droneStatus, missionData }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);
  const textareaRef = useRef(null);

  // Then instead of initializing in the component, move it outside
  const genAI = new GoogleGenerativeAI(GEMINI_API_KEY);

  useEffect(() => {
    if (messages.length === 0) {
      setMessages([{
        role: 'assistant',
        content: "Hello, I'm AURA, your AI Unified Rescue Assistant. I'm here to help with mission planning, drone operations, and emergency protocols. How can I assist you today?"
      }]);
    }
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const adjustTextareaHeight = () => {
    const textarea = textareaRef.current;
    if (textarea) {
      textarea.style.height = 'auto';
      textarea.style.height = `${Math.min(textarea.scrollHeight, 150)}px`;
    }
  };

  const handleInputChange = (e) => {
    setInput(e.target.value);
    adjustTextareaHeight();
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const handleSend = async () => {
    if (!input.trim() || isLoading) return;

    try {
        setIsLoading(true);
        const userMessage = { role: 'user', content: input };
        setMessages(prev => [...prev, userMessage]);

        // Log to check if API key is available
        console.log('API Key available:', !!GEMINI_API_KEY);

        const model = genAI.getGenerativeModel({ 
            model: 'gemini-pro'
        });

        // Simplify the chat initialization first
        const chat = await model.startChat();
        const result = await chat.sendMessage(input);
        const response = await result.response;
        
        setMessages(prev => [...prev, { 
            role: 'assistant', 
            content: response.text(),
            type: 'general'
        }]);
        
        setInput('');
        
    } catch (error) {
        console.error('Chat error:', error);
        setMessages(prev => [...prev, { 
            role: 'assistant', 
            content: `Error: ${error.message}`,
            type: 'error'
        }]);
    } finally {
        setIsLoading(false);
    }
};

  return (
    <div className="chat-interface">
      <div className="chat-icon" onClick={() => setIsOpen(!isOpen)}>
        <Bot />
      </div>
      
      <div className={`chat-panel ${isOpen ? 'open' : ''}`}>
        <div className="chat-header">
          <div className="aura-logo">
            <Bot />
          </div>
          <div className="header-text">
            <h3>AURA</h3>
            <small>AI Unified Rescue Assistant</small>
          </div>
          <button className="close-button" onClick={() => setIsOpen(false)}>
            <X />
          </button>
        </div>

        <div className="status-indicators">
          <div className="status-indicator">
            <BatteryMedium size={16} />
            <span>{droneStatus.battery}%</span>
          </div>
          <div className="status-indicator">
            <Signal size={16} />
            <span>{droneStatus.signalStrength}%</span>
          </div>
          <div className="status-indicator">
            <Cloud size={16} />
            <span>{missionData.weatherCondition}</span>
          </div>
        </div>

        <div className="chat-messages">
          {messages.map((message, index) => (
            <div key={index} className={`message ${message.role} ${message.type || ''}`}>
              {message.content}
            </div>
          ))}
          <div ref={messagesEndRef} />
        </div>

        <div className="chat-input">
          <div className="input-wrapper">
            <textarea
              ref={textareaRef}
              value={input}
              onChange={handleInputChange}
              onKeyPress={handleKeyPress}
              placeholder="Ask AURA about your mission..."
              disabled={isLoading}
              rows={1}
            />
          </div>
          <button 
            className="send-button"
            onClick={handleSend} 
            disabled={isLoading || !input.trim()}
          >
            {isLoading ? (
              <div className="loading-spinner" />
            ) : (
              <Send />
            )}
          </button>
        </div>
      </div>
    </div>
  );
};

export default ChatInterface;