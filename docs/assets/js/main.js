(() => {
  const root = document.documentElement;
  const header = document.querySelector('.site-header');
  const progress = document.querySelector('.page-progress span');
  const toggle = document.querySelector('.nav-toggle');
  const navLinks = document.querySelector('.nav-links');
  const sections = [...document.querySelectorAll('main section[id]')];
  const menuItems = [...document.querySelectorAll('.nav-links a[href^="#"]')];
  const prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  document.getElementById('year').textContent = new Date().getFullYear();

  const updateScrollState = () => {
    const scrollTop = window.scrollY;
    const scrollable = document.documentElement.scrollHeight - window.innerHeight;
    header.classList.toggle('scrolled', scrollTop > 20);
    progress.style.width = `${scrollable > 0 ? (scrollTop / scrollable) * 100 : 0}%`;

    let activeId = '';
    for (const section of sections) {
      if (section.getBoundingClientRect().top <= 150) activeId = section.id;
    }
    menuItems.forEach((item) => item.classList.toggle('active', item.hash === `#${activeId}`));
  };

  updateScrollState();
  window.addEventListener('scroll', updateScrollState, { passive: true });

  toggle.addEventListener('click', () => {
    const isOpen = toggle.getAttribute('aria-expanded') === 'true';
    toggle.setAttribute('aria-expanded', String(!isOpen));
    toggle.setAttribute('aria-label', isOpen ? 'Open navigation' : 'Close navigation');
    navLinks.classList.toggle('open', !isOpen);
  });

  navLinks.addEventListener('click', (event) => {
    if (!event.target.closest('a')) return;
    toggle.setAttribute('aria-expanded', 'false');
    toggle.setAttribute('aria-label', 'Open navigation');
    navLinks.classList.remove('open');
  });

  const revealItems = document.querySelectorAll('.reveal');
  if ('IntersectionObserver' in window && !prefersReducedMotion) {
    const revealObserver = new IntersectionObserver((entries, observer) => {
      entries.forEach((entry) => {
        if (!entry.isIntersecting) return;
        entry.target.classList.add('visible');
        observer.unobserve(entry.target);
      });
    }, { threshold: 0.12, rootMargin: '0px 0px -35px' });
    revealItems.forEach((item) => revealObserver.observe(item));
  } else {
    revealItems.forEach((item) => item.classList.add('visible'));
  }

  document.querySelector('.copy-button').addEventListener('click', async (event) => {
    const button = event.currentTarget;
    const text = document.getElementById(button.dataset.copy).innerText;
    try {
      await navigator.clipboard.writeText(text);
      button.textContent = 'Copied ✓';
    } catch (_error) {
      const selection = window.getSelection();
      const range = document.createRange();
      range.selectNodeContents(document.getElementById(button.dataset.copy));
      selection.removeAllRanges();
      selection.addRange(range);
      button.textContent = 'Selected';
    }
    window.setTimeout(() => { button.textContent = 'Copy'; }, 1800);
  });

  if (!prefersReducedMotion && window.matchMedia('(pointer: fine)').matches) {
    let frameRequested = false;
    window.addEventListener('pointermove', (event) => {
      if (frameRequested) return;
      frameRequested = true;
      window.requestAnimationFrame(() => {
        root.style.setProperty('--mouse-x', `${event.clientX}px`);
        root.style.setProperty('--mouse-y', `${event.clientY}px`);
        frameRequested = false;
      });
    }, { passive: true });
  }
})();
