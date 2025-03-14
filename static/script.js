document.addEventListener('keydown', function(e) {
  const allowedKeys = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
  if (allowedKeys.includes(e.key)) {
    fetch('/key_control', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ key: e.key })
    })
    .then(response => response.json())
    .then(data => {
      console.log('Key sent successfully:', data);
    })
    .catch(error => {
      console.error('Error sending key:', error);
    });

    // Add active class to the corresponding arrow element
    const arrowId = 'arrow-' + e.key.replace('Arrow', '').toLowerCase();
    const arrowElement = document.getElementById(arrowId);
    if (arrowElement) {
      arrowElement.classList.add('active');
    }
  }
});

document.addEventListener('keyup', function(e) {
  const allowedKeys = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
  if (allowedKeys.includes(e.key)) {
    // Remove the active class on key release
    const arrowId = 'arrow-' + e.key.replace('Arrow', '').toLowerCase();
    const arrowElement = document.getElementById(arrowId);
    if (arrowElement) {
      arrowElement.classList.remove('active');
    }
  }
});
