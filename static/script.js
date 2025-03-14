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
  }
});
