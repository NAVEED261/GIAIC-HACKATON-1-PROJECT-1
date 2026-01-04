import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import styles from './styles.module.css';

export default function UserProfile() {
  const { user, isAuthenticated, logout } = useAuth();
  const [showMenu, setShowMenu] = useState(false);
  const history = useHistory();

  if (!isAuthenticated || !user) {
    return (
      <div className={styles.authLinks}>
        <a href="/GIAIC-HACKATON-1-PROJECT-1/login" className={styles.loginBtn}>
          Sign In
        </a>
        <a href="/GIAIC-HACKATON-1-PROJECT-1/signup" className={styles.signupBtn}>
          Sign Up
        </a>
      </div>
    );
  }

  const handleLogout = () => {
    logout();
    history.push('/GIAIC-HACKATON-1-PROJECT-1/');
  };

  return (
    <div className={styles.profileContainer}>
      <button
        className={styles.profileButton}
        onClick={() => setShowMenu(!showMenu)}
        title={user.username}
      >
        <span className={styles.avatar}>{user.username.charAt(0).toUpperCase()}</span>
        <span className={styles.username}>{user.username}</span>
      </button>

      {showMenu && (
        <div className={styles.menu}>
          <div className={styles.menuItem} onClick={() => setShowMenu(false)}>
            <span className={styles.label}>User:</span>
            <span className={styles.value}>{user.username}</span>
          </div>
          <div className={styles.menuItem}>
            <span className={styles.label}>Email:</span>
            <span className={styles.value}>{user.email}</span>
          </div>
          <div className={styles.divider}></div>
          <button className={styles.logoutBtn} onClick={handleLogout}>
            Sign Out
          </button>
        </div>
      )}
    </div>
  );
}
