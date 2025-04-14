import { useState } from 'react';
import * as moduleBindings from '../module_bindings/index';

interface ConnectionPopupProps {
  onConnect: (connection: moduleBindings.DbConnection, identity: string) => void;
}

export function ConnectionPopup({ onConnect }: ConnectionPopupProps) {
  const [isConnecting, setIsConnecting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleConnect = async () => {
    if (isConnecting) return;
    setIsConnecting(true);
    setError(null);

    try {
      const connection = await moduleBindings.DbConnection.builder()
        /*.withUri("wss://maincloud.spacetimedb.com")*/ .withUri("http://localhost:3000")
        .withModuleName("vrchatdemo")
        .onConnect((ctx, identity) => {
          onConnect(ctx, identity.toHexString());
        })
        .onConnectError((ctx, error) => {
          console.error("Connection error:", error);
          setError(error.message);
          setIsConnecting(false);
        })
        .build();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to connect');
      setIsConnecting(false);
    }
  };

  return (
    <div style={{
      position: 'fixed',
      top: 0,
      left: 0,
      width: '100%',
      height: '100%',
      backgroundColor: 'rgba(0, 0, 0, 0.5)',
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'center',
      justifyContent: 'center',
      zIndex: 1000,
    }}>
      <h1 style={{
        fontSize: '3.5rem',
        fontWeight: 'bold',
        color: 'white',
        marginBottom: '2rem',
        animation: 'pulse 2s infinite',
        textShadow: '2px 2px 4px rgba(0, 0, 0, 0.5)'
      }}>
        VRChat Demo
      </h1>
      {error ? (
        <div style={{
          color: '#f87171',
          fontSize: '1.25rem',
          marginBottom: '1rem',
          textShadow: '1px 1px 2px rgba(0, 0, 0, 0.5)'
        }}>
          {error}
        </div>
      ) : isConnecting ? (
        <div style={{
          color: 'white',
          fontSize: '1.25rem',
          textShadow: '1px 1px 2px rgba(0, 0, 0, 0.5)'
        }}>
          Connecting...
        </div>
      ) : (
        <button
          onClick={handleConnect}
          style={{
            padding: '0.75rem 2rem',
            backgroundColor: 'rgba(59, 130, 246, 0.8)',
            color: 'white',
            fontSize: '1.25rem',
            fontWeight: '600',
            borderRadius: '0.5rem',
            border: 'none',
            cursor: 'pointer',
            transition: 'background-color 0.2s',
            textShadow: '1px 1px 2px rgba(0, 0, 0, 0.5)'
          }}
          onMouseOver={(e) => e.currentTarget.style.backgroundColor = 'rgba(37, 99, 235, 0.9)'}
          onMouseOut={(e) => e.currentTarget.style.backgroundColor = 'rgba(59, 130, 246, 0.8)'}
        >
          Connect
        </button>
      )}
    </div>
  );
} 